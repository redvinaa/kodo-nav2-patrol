"""patrol_executor - ROS 2 node for waypoint-based patrol execution."""

from __future__ import annotations

import math
from pathlib import Path
from typing import List, Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from patrol_interfaces.msg import PatrolState
from patrol_interfaces.srv import ListRoutes, StartPatrol, GetRoute
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger


class PatrolExecutor(Node):
    """Sends all route waypoints to Nav2 as a single NavigateThroughPoses goal."""

    def __init__(self) -> None:
        super().__init__("patrol_executor")

        self.declare_parameter("routes_dir", "")


        self._routes_dir = self.get_parameter("routes_dir").get_parameter_value().string_value
        if not self._routes_dir or not Path(self._routes_dir).is_dir():
            raise RuntimeError(
                f"Invalid 'routes_dir' parameter: '{self._routes_dir}' is not a directory."
            )
        # Patrol state
        self._state: int = PatrolState.IDLE
        self._route_name: str = ""
        self._waypoints: List[Pose2D] = []
        self._current_waypoint_index: int = 0
        self._goal_handle: Optional[ClientGoalHandle] = None

        self._nav_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(PatrolState, "patrol/state", latched_qos)

        # Service servers
        self.create_service(StartPatrol, "patrol/start", self._on_start)
        self.create_service(Trigger, "patrol/stop", self._on_stop)
        self.create_service(
            ListRoutes, "patrol/list_routes", self._on_list_routes
        )
        self.create_service(
            GetRoute, "patrol/get_route", self._on_get_route
        )

        self._publish_state()
        self.get_logger().info("PatrolExecutor ready.")

    # Helpers

    def _load_waypoints(self, route_name: str) -> List[Pose2D]:
        route_file = Path(self._routes_dir) / f"{route_name}.yaml"
        if not route_file.exists():
            raise FileNotFoundError(f"Route file not found: {route_file}")
        with open(route_file) as fh:
            data = yaml.safe_load(fh)
        raw = data.get("waypoints", [])
        if not raw:
            raise ValueError(f"Route '{route_name}' contains no waypoints.")
        waypoints: List[Pose2D] = []
        for i, wp in enumerate(raw):
            try:
                pose = Pose2D(x=float(wp["x"]), y=float(wp["y"]), theta=float(wp.get("yaw", 0.0)))
                waypoints.append(pose)
            except KeyError as exc:
                raise ValueError(
                    f"Waypoint {i} in route '{route_name}' is missing field: {exc}"
                ) from exc
        return waypoints

    def _set_state(self, state: int) -> None:
        self._state = state
        self._publish_state()

    def _publish_state(self) -> None:
        msg = PatrolState()
        msg.state = self._state
        msg.route_name = self._route_name
        msg.n_waypoints = len(self._waypoints)
        msg.current_waypoint_index = self._current_waypoint_index
        self._state_pub.publish(msg)

    def _cancel_goal(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

    # Service handlers

    def _on_get_route(
        self, request: GetRoute.Request, response: GetRoute.Response
    ) -> GetRoute.Response:
        try:
            waypoints = self._load_waypoints(request.route_name)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            response.waypoints = []
            return response
        response.success = True
        response.message = ""
        response.waypoints = waypoints
        return response

    def _on_list_routes(
        self, _: ListRoutes.Request, response: ListRoutes.Response
    ) -> ListRoutes.Response:
        response.route_names = sorted(p.stem for p in Path(self._routes_dir).glob("*.yaml"))
        return response

    def _on_start(
        self, request: StartPatrol.Request, response: StartPatrol.Response
    ) -> StartPatrol.Response:
        self._cancel_goal()
        try:
            self._waypoints = self._load_waypoints(request.route_name)
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        self._route_name = request.route_name
        self._set_state(PatrolState.RUNNING)
        if not self._nav_client.server_is_ready():
            self.get_logger().error("navigate_through_poses action server not available.")
            self._set_state(PatrolState.FAILED)
            response.success = False
            response.message = "Nav2 action server not available."
            return response
        self._dispatch_goal()
        response.success = True
        response.message = f"Patrol started on route '{request.route_name}'."
        return response

    def _on_stop(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._cancel_goal()
        self._route_name = ""
        self._waypoints = []
        self._current_waypoint_index = 0
        self._set_state(PatrolState.IDLE)
        response.success = True
        response.message = "Patrol stopped."
        return response

    # Navigation

    def _dispatch_goal(self) -> None:
        goal = NavigateThroughPoses.Goal()
        for wp in self._waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.orientation.z = math.sin(wp.theta / 2.0)
            pose.pose.orientation.w = math.cos(wp.theta / 2.0)
            goal.poses.append(pose)
        self.get_logger().info(
            f"Sending {len(goal.poses)} waypoint(s) for route '{self._route_name}'."
        )
        future = self._nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_accepted)

    def _on_feedback(self, feedback_msg) -> None:
        remaining = feedback_msg.feedback.number_of_poses_remaining
        self._current_waypoint_index = len(self._waypoints) - remaining
        self._publish_state()

    def _on_goal_accepted(self, future) -> None:
        handle: ClientGoalHandle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("NavigateThroughPoses goal rejected.")
            self._set_state(PatrolState.FAILED)
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self._goal_handle = None
        if self._state != PatrolState.RUNNING:
            return
        result = future.result()
        status = result.status if result is not None else GoalStatus.STATUS_UNKNOWN
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("All waypoints reached. Patrol COMPLETED.")
            self._set_state(PatrolState.COMPLETED)
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("Navigation cancelled externally. Patrol PAUSED.")
            self._set_state(PatrolState.PAUSED)
        else:
            self.get_logger().warning(f"NavigateThroughPoses ended with status {status}.")
            self._set_state(PatrolState.FAILED)



def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
