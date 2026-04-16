"""
patrol_system.launch.py — single entry-point for the full patrol stack.

Includes sim_nav.launch.py (Gazebo + Nav2 + RViz) and adds:
  - patrol_executor node (waypoint patrol)
  - patrol_web_server node (optional, set launch_web_interface:=true)

Usage
-----
ros2 launch patrol_bringup patrol_system.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    patrol_bringup_dir = get_package_share_directory("patrol_bringup")
    web_interface_dir = get_package_share_directory("patrol_web_interface")

    use_sim_time = LaunchConfiguration("use_sim_time")
    routes_dir = LaunchConfiguration("routes_dir")
    launch_web = LaunchConfiguration("launch_web_interface")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")

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
    declare_launch_web_cmd = DeclareLaunchArgument(
        "launch_web_interface",
        default_value="true",
        description="Launch the patrol web interface server",
    )
    declare_web_host_cmd = DeclareLaunchArgument(
        "web_host",
        default_value="0.0.0.0",
        description="Host address for the web server",
    )
    declare_web_port_cmd = DeclareLaunchArgument(
        "web_port",
        default_value="8080",
        description="Port for the web server",
    )

    sim_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(patrol_bringup_dir, "launch", "sim_nav.launch.py")
        )
    )

    web_server_cmd = Node(
        package="patrol_web_interface",
        executable="web_server",
        name="patrol_web_server",
        output="screen",
        parameters=[
            {
                "host": web_host,
                "port": web_port,
                "routes_dir": routes_dir,
                "web_dir": os.path.join(web_interface_dir, "web"),
            }
        ],
        condition=IfCondition(launch_web),
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
    ld.add_action(declare_launch_web_cmd)
    ld.add_action(declare_web_host_cmd)
    ld.add_action(declare_web_port_cmd)

    ld.add_action(sim_nav_cmd)
    ld.add_action(patrol_executor_cmd)
    ld.add_action(web_server_cmd)

    return ld
