# kodo-nav2-patrol

A modular ROS 2 Humble workspace for goal-based robot navigation using TurtleBot3,
Gazebo, and the Nav2 stack — all running inside a reproducible Docker environment.

## Package Structure

```
src/
├── patrol_bringup/       # Launch files & RViz config — single entry point
├── patrol_simulation/    # Gazebo world and spawn configuration
└── patrol_navigation/    # Nav2 params, map files, localization config
```

## Prerequisites

- Docker (no ROS installation required on the host)

## Setup

### 1. Allow X11 forwarding from Docker

```bash
xhost +local:docker
```

### 2. Build the Docker image

```bash
docker compose build
```

## Usage

### Start the container

```bash
docker compose up kodo
```

`setup_env.bash` is sourced automatically on login.

### Build stack

```bash
colcon build
```

After first build, you have to source the environment again.

### Launch the full system (single command)

```bash
ros2 launch patrol_bringup sim_nav.launch.py
```

Starts Gazebo, spawns the robot, brings up Nav2, and opens RViz. Gazebo runs headless by default

### Sending a navigation goal

**Via RViz (preferred):**
1. Click the **"2D Goal Pose"** button in the RViz toolbar
2. Click and drag on the map to set the goal position and orientation
3. The robot will plan a path and navigate to the goal

**Via CLI:**

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"
```

## Assumptions & Limitations

- **Map**: Pre-built map is included in `src/patrol_navigation/maps/`.
- **Localization**: Uses AMCL. If the estimated pose drifts, use **"2D Pose Estimate"** in RViz.
- **Display**: Requires an X11 server on the host. On headless servers, use a virtual display (e.g., `Xvfb`).
- **Robot model**: Defaults to TurtleBot3 Waffle. Change `TURTLEBOT3_MODEL` in `setup_env.bash` to use `burger`.

