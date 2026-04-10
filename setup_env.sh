#!/usr/bin/env bash
# setup_env.sh
# Source this file from .bashrc for a fully configured ROS 2 + workspace environment.
# It is automatically sourced on login inside the Docker container.

# Source ROS 2 Humble base environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local workspace overlay (if already built)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Turtlebot3 model selection (waffle provides lidar + camera)
export TURTLEBOT3_MODEL=waffle

# ROS domain isolation
export ROS_DOMAIN_ID=0

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Gazebo model path so turtlebot3 models are found
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:+$GAZEBO_MODEL_PATH:}/opt/ros/humble/share/turtlebot3_gazebo/models

echo "[setup_env] ROS 2 Humble environment loaded. TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
