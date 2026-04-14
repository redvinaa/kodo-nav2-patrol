"""
patrol_system.launch.py — single entry-point for the full patrol stack.

Starts:
  1. Gazebo simulation (gzserver + optional gzclient)
  2. Robot state publisher
  3. Nav2 bringup (AMCL, costmaps, planners, etc.)
  4. RViz (optional)
  5. patrol_executor node (waypoint patrol)

Usage
-----
ros2 launch patrol_bringup patrol_system.launch.py

Optional overrides (same as sim_nav.launch.py):
  route_name:=demo_route
  routes_dir:=/workspace/routes
  max_retries:=3
  use_rviz:=True
  headless:=True
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # ------------------------------------------------------------------ dirs
    bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_dir = os.path.join(bringup_dir, "launch")

    simulation_dir = get_package_share_directory("patrol_simulation")
    navigation_dir = get_package_share_directory("patrol_navigation")
    patrol_bringup_dir = get_package_share_directory("patrol_bringup")

    # ------------------------------------------------------------------ configs
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_simulator = LaunchConfiguration("use_simulator")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    pose = {
        "x": LaunchConfiguration("x_pose", default="-2.00"),
        "y": LaunchConfiguration("y_pose", default="-0.50"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }

    # Patrol-specific
    routes_dir = LaunchConfiguration("routes_dir")
    max_retries = LaunchConfiguration("max_retries")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ------------------------------------------------------------------ args
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )
    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether to run SLAM"
    )
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(navigation_dir, "maps", "map.yaml"),
        description="Full path to map YAML to load",
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(navigation_dir, "params", "nav2_params.yaml"),
        description="Full path to the Nav2 parameters file",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Autostart Nav2 lifecycle nodes"
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="True", description="Use composed bringup"
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Respawn nodes on crash (non-composed only)",
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(patrol_bringup_dir, "rviz", "patrol.rviz"),
        description="Full path to the RViz config file",
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="True", description="Start Gazebo simulator"
    )
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Start robot_state_publisher",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Start RViz"
    )
    declare_headless_cmd = DeclareLaunchArgument(
        "headless", default_value="True", description="Run Gazebo headless (no GUI)"
    )
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(simulation_dir, "worlds", "world_only.model"),
        description="Full path to the Gazebo world file",
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot3_waffle", description="Robot entity name"
    )
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(bringup_dir, "worlds", "waffle.model"),
        description="Full path to the robot SDF to spawn",
    )
    # Patrol parameters
    declare_routes_dir_cmd = DeclareLaunchArgument(
        "routes_dir",
        description="Directory containing route YAML files",
    )
    declare_max_retries_cmd = DeclareLaunchArgument(
        "max_retries",
        default_value="3",
        description="Maximum navigation retries per waypoint before FAILED",
    )

    # ------------------------------------------------------------------ actions
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            world,
        ],
        cwd=[nav2_launch_dir],
        output="screen",
    )

    start_gazebo_client_cmd = ExecuteProcess(
        condition=IfCondition(
            PythonExpression([use_simulator, " and not ", headless])
        ),
        cmd=["gzclient"],
        cwd=[nav2_launch_dir],
        output="screen",
    )

    urdf = os.path.join(bringup_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf, "r") as fh:
        robot_description = fh.read()

    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
        remappings=remappings,
    )

    start_gazebo_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            robot_name,
            "-file",
            robot_sdf,
            "-robot_namespace",
            namespace,
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "rviz_launch.py")
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": autostart,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
        }.items(),
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
                "max_retries": max_retries,
            }
        ],
    )

    # ------------------------------------------------------------------ ld
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_routes_dir_cmd)
    ld.add_action(declare_max_retries_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(patrol_executor_cmd)

    return ld
