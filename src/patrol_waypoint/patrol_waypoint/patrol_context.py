"""Patrol navigation context dataclass."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from rclpy.action.client import ClientGoalHandle
from geometry_msgs.msg import Pose2D


@dataclass
class PatrolContext:
    """Mutable execution context that persists across pause/resume cycles."""

    # Name of the currently loaded route (stem of the YAML file).
    route_name: str = ""
    # Ordered list of waypoints for the active route.
    waypoints: List[Pose2D] = field(default_factory=list)
    # Index of the waypoint that is currently being (or will be) navigated to.
    current_index: int = 0
    # Number of consecutive failures on the current waypoint.
    retry_count: int = 0
    # Active Nav2 action goal handle; None when no goal is in-flight.
    goal_handle: Optional[ClientGoalHandle] = None

    @property
    def n_waypoints(self) -> int:
        return len(self.waypoints)

    @property
    def is_finished(self) -> bool:
        return self.current_index >= self.n_waypoints

    def reset(self) -> None:
        """Reset to a clean slate (keeps route/waypoints for re-use)."""
        self.current_index = 0
        self.retry_count = 0
        self.goal_handle = None
