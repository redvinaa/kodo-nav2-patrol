"""
patrol_executor — ROS 2 node for waypoint-based patrol execution.

Services
--------
/patrol/start   (patrol_interfaces/srv/StartPatrol)
/patrol/stop    (std_srvs/srv/Trigger)
/patrol/pause   (std_srvs/srv/Trigger)
/patrol/resume  (std_srvs/srv/Trigger)

Published Topics
----------------
/patrol/state   (patrol_interfaces/msg/PatrolState)
"""

from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from patrol_interfaces.msg import PatrolState
from patrol_interfaces.srv import StartPatrol
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from patrol_waypoint.patrol_context import PatrolContext


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs Quaternion."""
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _build_pose_stamped(waypoint: Pose2D, frame_id: str = "map") -> PoseStamped:
    """Build a PoseStamped message from a Pose2D."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = waypoint.x
    pose.pose.position.y = waypoint.y
    pose.pose.position.z = 0.0
    pose.pose.orientation = _yaw_to_quaternion(waypoint.theta)
    return pose


class PatrolExecutor(Node):
    """Executes a sequence of Nav2 waypoints, exposing control services."""

    def __init__(self) -> None:
        super().__init__("patrol_executor")

        # Parameters
        self.declare_parameter("routes_dir", "")
        self.declare_parameter("max_retries", 3)
        self.declare_parameter("use_sim_time", True)

        self._routes_dir: str = (
            self.get_parameter("routes_dir").get_parameter_value().string_value
        )
        if not self._routes_dir:
            raise RuntimeError(
                "Parameter 'routes_dir' must be set. "
                "Pass routes_dir:=<path> at launch or via a parameter file."
            )
        self._max_retries: int = (
            self.get_parameter("max_retries").get_parameter_value().integer_value
        )

        self.get_logger().info(f"Routes directory: {self._routes_dir}")
        self.get_logger().info(f"Max retries per waypoint: {self._max_retries}")

        # State
        self._state: int = PatrolState.IDLE
        self._ctx: PatrolContext = PatrolContext()
        self._lock = threading.Lock()

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Publishers
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
    PatrolState, "patrol/state", latched_qos
        )

        # Service servers
        self._start_srv = self.create_service(
            StartPatrol, "patrol/start", self._handle_start
        )
        self._stop_srv = self.create_service(
            Trigger, "patrol/stop", self._handle_stop
        )
        self._pause_srv = self.create_service(
            Trigger, "patrol/pause", self._handle_pause
        )
        self._resume_srv = self.create_service(
            Trigger, "patrol/resume", self._handle_resume
        )

        # Initial state publish
        self._publish_state()
        self.get_logger().info("PatrolExecutor ready.")

    # Utility
    def _load_route(self, route_name: str) -> list[Pose2D]:
        """Load and parse a route YAML file; raise on any error."""
        route_file = Path(self._routes_dir) / f"{route_name}.yaml"
        self.get_logger().info(f"Loading route from: {route_file}")

        if not route_file.exists():
            raise FileNotFoundError(f"Route file not found: {route_file}")

        with open(route_file, "r") as fh:
            data = yaml.safe_load(fh)

        raw_waypoints = data.get("waypoints", [])
        if not raw_waypoints:
            raise ValueError(f"Route '{route_name}' contains no waypoints.")

        waypoints: list[Pose2D] = []
        for i, wp in enumerate(raw_waypoints):
            try:
                pose = Pose2D()
                pose.x = float(wp["x"])
                pose.y = float(wp["y"])
                pose.theta = float(wp.get("yaw", 0.0))
                waypoints.append(pose)
            except KeyError as exc:
                raise ValueError(
                    f"Waypoint {i} in route '{route_name}' is missing field: {exc}"
                ) from exc

        self.get_logger().info(
            f"Loaded route '{route_name}' with {len(waypoints)} waypoint(s)."
        )
        return waypoints

    # State management
    def _set_state(self, new_state: int) -> None:
        self.get_logger().info(
            f"State transition: {self._state} -> {new_state}"
        )
        self._state = new_state
        self._publish_state()

    def _publish_state(self) -> None:
        msg = PatrolState()
        msg.state = self._state
        msg.current_waypoint_index = self._ctx.current_index
        msg.n_waypoints = self._ctx.n_waypoints
        msg.route_name = self._ctx.route_name
        self._status_pub.publish(msg)

    # Service handlers
    def _handle_start(
        self, request: StartPatrol.Request, response: StartPatrol.Response
    ) -> StartPatrol.Response:
        with self._lock:
            self.get_logger().info(
                f"Start requested for route: '{request.route_name}'"
            )

            # Cancel any running goal first
            if self._state in (PatrolState.RUNNING, PatrolState.PAUSED):
                self._cancel_current_goal()

            # Load route (fresh load on every start)
            try:
                waypoints = self._load_route(request.route_name)
            except Exception as exc:
                msg = f"Failed to load route '{request.route_name}': {exc}"
                self.get_logger().error(msg)
                response.success = False
                response.message = msg
                return response

            # Initialise context
            self._ctx = PatrolContext(
                route_name=request.route_name,
                waypoints=waypoints,
            )
            self._set_state(PatrolState.RUNNING)

            # Kick off navigation in a separate thread so the service returns
            threading.Thread(target=self._run_patrol, daemon=True).start()

            response.success = True
            response.message = (
                f"Patrol started: route '{request.route_name}', "
                f"{self._ctx.n_waypoints} waypoint(s)."
            )
            return response

    def _handle_stop(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            self.get_logger().info("Stop requested.")
            if self._state == PatrolState.IDLE:
                response.success = True
                response.message = "Already idle."
                return response

            self._cancel_current_goal()
            self._ctx = PatrolContext()
            self._set_state(PatrolState.IDLE)

            response.success = True
            response.message = "Patrol stopped. System is IDLE."
            return response

    def _handle_pause(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            self.get_logger().info("Pause requested.")
            if self._state != PatrolState.RUNNING:
                response.success = False
                response.message = (
                    f"Cannot pause: current state is '{self._state}'."
                )
                return response

            self._cancel_current_goal()
            self._set_state(PatrolState.PAUSED)

            response.success = True
            response.message = (
                f"Patrol paused at waypoint {self._ctx.current_index} "
                f"/ {self._ctx.n_waypoints}."
            )
            return response

    def _handle_resume(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._lock:
            self.get_logger().info("Resume requested.")
            if self._state != PatrolState.PAUSED:
                response.success = False
                response.message = (
                    f"Cannot resume: current state is '{self._state}'."
                )
                return response

            self._set_state(PatrolState.RUNNING)

            # Resume from the stored index
            threading.Thread(target=self._run_patrol, daemon=True).start()

            response.success = True
            response.message = (
                f"Patrol resumed from waypoint {self._ctx.current_index} "
                f"/ {self._ctx.n_waypoints}."
            )
            return response

    # Navigation execution
    def _cancel_current_goal(self) -> None:
        """Cancel the in-flight Nav2 goal synchronously (best-effort)."""
        handle: Optional[ClientGoalHandle] = self._ctx.goal_handle
        if handle is None:
            return
        self.get_logger().info("Cancelling current navigation goal…")
        try:
            cancel_future = handle.cancel_goal_async()
            # Give it up to 5 s to process the cancel
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Goal cancellation raised: {exc}")
        finally:
            self._ctx.goal_handle = None

    def _run_patrol(self) -> None:
        """
        Sequential waypoint execution loop.

        Runs in a dedicated daemon thread so service callbacks return
        immediately.  The loop checks ``self._state`` after every goal
        to honour pause/stop requests.
        """
        self.get_logger().info(
            f"Patrol loop starting at waypoint {self._ctx.current_index} "
            f"/ {self._ctx.n_waypoints}."
        )

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "navigate_to_pose action server not available after 10 s."
            )
            with self._lock:
                self._set_state(PatrolState.FAILED)
            return

        while True:
            # Check if we should keep going
            with self._lock:
                if self._state != PatrolState.RUNNING:
                    self.get_logger().info(
                        f"Patrol loop exiting (state={self._state})."
                    )
                    return
                if self._ctx.is_finished:
                    self.get_logger().info(
                        "All waypoints completed. Patrol COMPLETED."
                    )
                    self._set_state(PatrolState.COMPLETED)
                    return
                waypoint = self._ctx.waypoints[self._ctx.current_index]
                index = self._ctx.current_index

            # Build and send goal
            self.get_logger().info(
                f"Navigating to waypoint {index + 1}/{self._ctx.n_waypoints}: "
                f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, theta={waypoint.theta:.2f}"
            )

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = _build_pose_stamped(waypoint)
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

            send_future = self._nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future)

            goal_handle: ClientGoalHandle = send_future.result()

            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error(
                    f"Goal for waypoint {index + 1} was rejected by Nav2."
                )
                if self._handle_waypoint_failure(index):
                    continue  # retry
                return  # FAILED state already set

            with self._lock:
                self._ctx.goal_handle = goal_handle
                self._publish_state()

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            with self._lock:
                self._ctx.goal_handle = None

                # State may have changed while we were navigating
                if self._state != PatrolState.RUNNING:
                    self.get_logger().info(
                        f"Patrol loop exiting after goal result "
                        f"(state={self._state})."
                    )
                    return

            result = result_future.result()
            status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN

            if status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f"Waypoint {index + 1}/{self._ctx.n_waypoints} reached successfully."
                )
                with self._lock:
                    self._ctx.current_index += 1
                    self._ctx.retry_count = 0
                    self._publish_state()
            else:
                self.get_logger().warning(
                    f"Waypoint {index + 1} navigation ended with status {status}."
                )
                if self._handle_waypoint_failure(index):
                    continue  # retry
                return  # FAILED state already set

    def _handle_waypoint_failure(self, index: int) -> bool:
        """
        Increment retry counter and decide whether to retry.

        Returns

        bool
            ``True`` if the caller should retry the same waypoint,
            ``False`` if the retry limit was reached (state set to FAILED).
        """
        with self._lock:
            self._ctx.retry_count += 1
            if self._ctx.retry_count <= self._max_retries:
                self.get_logger().warning(
                    f"Retrying waypoint {index + 1} "
                    f"(attempt {self._ctx.retry_count}/{self._max_retries})…"
                )
                return True
            else:
                self.get_logger().error(
                    f"Waypoint {index + 1} failed after {self._max_retries} retries. "
                    f"Setting state to FAILED."
                )
                self._set_state(PatrolState.FAILED)
                return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
