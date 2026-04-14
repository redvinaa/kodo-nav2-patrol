#!/usr/bin/env bash

# Source ROS 2 Humble base environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source local workspace overlay (if already built)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Gazebo model path so turtlebot3 models are found
export GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH}:/opt/ros/humble/share/turtlebot3_gazebo/models:/ros2_ws/gazebo_models"

# Force software rendering so Gazebo works without a real GPU / proper GLX
export LIBGL_ALWAYS_SOFTWARE=1

echo "[setup_env] ROS 2 Humble environment loaded."
