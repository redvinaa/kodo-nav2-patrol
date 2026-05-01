"""
patrol_system.launch.py — single entry-point for the full patrol stack.

Includes sim_nav.launch.py (Gazebo + Nav2 + RViz) and adds:
  - patrol_executor node (waypoint patrol)

Usage
-----
ros2 launch patrol_bringup patrol_system.launch.py

Optional overrides (forwarded to sim_nav.launch.py):
  map:=...
  params_file:=...
  use_rviz:=True
  headless:=True
  ... (any sim_nav argument)

Patrol-specific overrides:
  routes_dir:=/ros2_ws/routes
  max_retries:=3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    patrol_bringup_dir = get_package_share_directory("patrol_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    routes_dir = LaunchConfiguration("routes_dir")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_routes_dir_cmd = DeclareLaunchArgument(
        "routes_dir",
        default_value="/ros2_ws/routes",
        description="Directory containing route YAML files",
    )

    sim_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(patrol_bringup_dir, "launch", "sim_nav.launch.py")
        )
    )

    patrol_executor_cmd = Node(
        package="patrol_waypoint",
        executable="patrol_executor",
        name="patrol_executor",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "routes_dir": routes_dir,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_routes_dir_cmd)

    ld.add_action(sim_nav_cmd)
    ld.add_action(patrol_executor_cmd)

    return ld
