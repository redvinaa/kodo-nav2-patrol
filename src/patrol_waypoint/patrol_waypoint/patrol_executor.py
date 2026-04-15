"""patrol_executor - ROS 2 node for waypoint-based patrol execution."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose
from patrol_interfaces.msg import PatrolState
from patrol_interfaces.srv import StartPatrol
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from patrol_waypoint.patrol_context import PatrolContext


class PatrolExecutor(Node):
    """Executes a sequence of Nav2 waypoints, exposing control services."""

    def __init__(self) -> None:
        super().__init__("patrol_executor")

        # Parameters
        self.declare_parameter("routes_dir", "")
        self.declare_parameter("max_retries", 3)

        self._routes_dir: str = (
            self.get_parameter("routes_dir").get_parameter_value().string_value
        )
        if not self._routes_dir or not Path(self._routes_dir).is_dir():
            raise RuntimeError(
                f"Invalid 'routes_dir' parameter: '{self._routes_dir}' is not a directory."
            )
        self._max_retries: int = (
            self.get_parameter("max_retries").get_parameter_value().integer_value
        )

        self.get_logger().info(f"Routes directory: {self._routes_dir}")
        self.get_logger().info(f"Max retries per waypoint: {self._max_retries}")

        # State
        self._state: int = PatrolState.IDLE
        self._ctx: PatrolContext = PatrolContext()

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Publishers
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(PatrolState, "patrol/state", latched_qos)

        # Service servers
        self._start_srv = self.create_service(StartPatrol, "patrol/start", self._handle_start)
        self._stop_srv = self.create_service(Trigger, "patrol/stop", self._handle_stop)
        self._pause_srv = self.create_service(Trigger, "patrol/pause", self._handle_pause)
        self._resume_srv = self.create_service(Trigger, "patrol/resume", self._handle_resume)

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
                    f"Waypoint {i} in route '{route_name}' is missing field: {exc}, ignoring."
                ) from exc

        self.get_logger().info(
            f"Loaded route '{route_name}' with {len(waypoints)} waypoint(s)."
        )
        return waypoints

    # State management

    def _set_state(self, new_state: int) -> None:
        self.get_logger().info(f"State transition: {self._state} -> {new_state}")
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
        self.get_logger().info(f"Start requested for route: '{request.route_name}'")

        if self._state in (PatrolState.RUNNING, PatrolState.PAUSED):
            self._cancel_current_goal()

        try:
            waypoints = self._load_route(request.route_name)
        except Exception as exc:
            msg = f"Failed to load route '{request.route_name}': {exc}"
            self.get_logger().error(msg)
            response.success = False
            response.message = msg
            return response

        self._ctx = PatrolContext(route_name=request.route_name, waypoints=waypoints)
        self._set_state(PatrolState.RUNNING)
        self._send_next_goal()

        response.success = True
        response.message = (
            f"Patrol started: route '{request.route_name}', "
            f"{self._ctx.n_waypoints} waypoint(s)."
        )
        return response

    def _handle_stop(
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
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
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("Pause requested.")
        if self._state != PatrolState.RUNNING:
            response.success = False
            response.message = f"Cannot pause: current state is '{self._state}'."
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
        self, _: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("Resume requested.")
        if self._state != PatrolState.PAUSED:
            response.success = False
            response.message = f"Cannot resume: current state is '{self._state}'."
            return response

        self._set_state(PatrolState.RUNNING)
        self._send_next_goal()

        response.success = True
        response.message = (
            f"Patrol resumed from waypoint {self._ctx.current_index} "
            f"/ {self._ctx.n_waypoints}."
        )
        return response

    # Navigation execution

    def _send_next_goal(self) -> None:
        """Send the goal for the current waypoint index, or finish if done."""
        if self._state != PatrolState.RUNNING:
            return

        if self._ctx.is_finished:
            self.get_logger().info("All waypoints completed. Patrol COMPLETED.")
            self._set_state(PatrolState.COMPLETED)
            return

        if not self._nav_client.server_is_ready():
            self.get_logger().warning(
                "navigate_to_pose action server not ready, retrying in 1 s…"
            )
            self._server_retry_timer = self.create_timer(1.0, self._on_server_ready_retry)
            return

        self._dispatch_goal()

    def _on_server_ready_retry(self) -> None:
        """One-shot timer callback: retry sending goal once server is ready."""
        self.destroy_timer(self._server_retry_timer)
        self._send_next_goal()

    def _dispatch_goal(self) -> None:
        """Build and send the Nav2 goal for the current waypoint."""
        waypoint = self._ctx.waypoints[self._ctx.current_index]
        index = self._ctx.current_index

        self.get_logger().info(
            f"Navigating to waypoint {index + 1}/{self._ctx.n_waypoints}: "
            f"x={waypoint.x:.2f}, y={waypoint.y:.2f}, theta={waypoint.theta:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint.x
        goal_msg.pose.pose.position.y = waypoint.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(waypoint.theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(waypoint.theta / 2.0)

        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        """Called when Nav2 responds to the goal request."""
        goal_handle: ClientGoalHandle = future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f"Goal for waypoint {self._ctx.current_index + 1} was rejected by Nav2."
            )
            self._handle_waypoint_failure()
            return

        self._ctx.goal_handle = goal_handle
        self._publish_state()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        """Called when the Nav2 goal finishes (succeeded, failed, or cancelled)."""
        self._ctx.goal_handle = None

        # Ignore result if state changed (stop/pause already handled)
        if self._state != PatrolState.RUNNING:
            return

        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f"Waypoint {self._ctx.current_index + 1}/{self._ctx.n_waypoints} "
                f"reached successfully."
            )
            self._ctx.current_index += 1
            self._ctx.retry_count = 0
            self._publish_state()
            self._send_next_goal()
        else:
            self.get_logger().warning(
                f"Waypoint {self._ctx.current_index + 1} navigation ended "
                f"with status {status}."
            )
            self._handle_waypoint_failure()

    def _handle_waypoint_failure(self) -> None:
        """Increment retry counter and retry or set FAILED."""
        self._ctx.retry_count += 1
        index = self._ctx.current_index

        if self._ctx.retry_count <= self._max_retries:
            self.get_logger().warning(
                f"Retrying waypoint {index + 1} "
                f"(attempt {self._ctx.retry_count}/{self._max_retries})…"
            )
            self._dispatch_goal()
        else:
            self.get_logger().error(
                f"Waypoint {index + 1} failed after {self._max_retries} retries. "
                f"Setting state to FAILED."
            )
            self._set_state(PatrolState.FAILED)

    def _cancel_current_goal(self) -> None:
        """Fire-and-forget cancel of the in-flight Nav2 goal."""
        if self._ctx.goal_handle is None:
            return
        self.get_logger().info("Cancelling current navigation goal.")
        cancel_future = self._ctx.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._on_goal_cancelled)
        self._ctx.goal_handle = None

    def _on_goal_cancelled(self, future) -> None:
        """Called when Nav2 acknowledges the cancel request."""
        self.get_logger().info("Navigation goal cancelled.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
