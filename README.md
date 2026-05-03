# kodo-nav2-patrol

A modular ROS 2 Humble workspace for waypoint-based robot patrol using TurtleBot3,
Gazebo, and the Nav2 stack — all running inside a reproducible Docker environment.

## Package Structure

```
src/
├── patrol_bringup/       # Launch files & RViz config — single entry point
├── patrol_bt_plugins/    # Custom BehaviorTree.CPP nodes (pause/resume control)
├── patrol_interfaces/    # Custom ROS 2 msg/srv definitions
├── patrol_navigation/    # Nav2 params, map files, localization config
├── patrol_simulation/    # Gazebo world and spawn configuration
├── patrol_waypoint/      # Waypoint loading, execution logic, state management
└── patrol_web_interface/ # aiohttp web server — browser-based patrol control UI

routes/
├── demo_route.yaml       # Short 4-point patrol route
└── perimeter_route.yaml  # Larger 5-point perimeter patrol
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

### Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### Launch the full patrol system (single command)

```bash
ros2 launch patrol_bringup patrol_system.launch.py
```

This starts Gazebo (headless), spawns the robot, brings up Nav2, opens RViz,
starts the `patrol_executor` node, and launches the web interface on port 8080.

Optional overrides:

```bash
ros2 launch patrol_bringup patrol_system.launch.py \
  routes_dir:=/workspace/routes \
  use_rviz:=True \
  headless:=False \
  launch_web_interface:=true \
  web_host:=0.0.0.0 \
  web_port:=8080
```

## Web Interface

When the system is running, open a browser and navigate to:

```
http://localhost:8080
```

The UI provides:

- **Live map view** — occupancy grid with robot pose, planned global path, and route waypoints overlaid on a canvas
- **Route selector** — choose any loaded route from a dropdown
- **Patrol controls** — Start / Pause / Resume / Stop buttons
- **Status bar** — live patrol state, active route name, and current waypoint index

The web server can also be launched independently:

```bash
ros2 launch patrol_web_interface web_interface.launch.py \
  host:=0.0.0.0 \
  port:=8080 \
  routes_dir:=/ros2_ws/routes
```

### Web server endpoints

| Method | Path | Description |
|---|---|---|
| `GET` | `/map` | Map image (PNG, converted from PGM) |
| `GET` | `/api/map_info` | Map metadata (resolution, origin, size) |
| `GET` | `/api/list_routes` | List available route names |
| `GET` | `/api/get_route?name=<n>` | Waypoints for a named route |
| `POST` | `/api/start` | Start patrol `{"route_name": "..."}` |
| `POST` | `/api/stop` | Stop patrol |
| `POST` | `/api/pause` | Pause patrol |
| `POST` | `/api/resume` | Resume patrol |
| `WS` | `/ws` | Live stream of patrol state, robot pose, and global path |

### Starting / stopping patrol

**Start patrol** (loads route fresh on every call):

```bash
ros2 service call /patrol/start patrol_interfaces/srv/StartPatrol \
  "{route_name: 'demo_route'}"
```

**Pause patrol** (stores current waypoint index):

```bash
ros2 service call /patrol/pause std_srvs/srv/Trigger "{}"
```

**Resume patrol** (continues from stored index):

```bash
ros2 service call /patrol/resume std_srvs/srv/Trigger "{}"
```

**Stop patrol** (cancels goal, resets to IDLE):

```bash
ros2 service call /patrol/stop std_srvs/srv/Trigger "{}"
```

**List available routes**:

```bash
ros2 service call /patrol/list_routes patrol_interfaces/srv/ListRoutes
```

### Monitoring patrol state

```bash
ros2 topic echo /patrol/state
```

Published fields:

| Field | Type | Description |
|---|---|---|
| `state` | uint32 | Enum constant, see `patrol_interfaces/msg/PatrolState.msg` |
| `route_name` | string | Active route name |
| `current_waypoint_index` | int32 | Zero-based index of current waypoint |
| `n_waypoints` | int32 | Total waypoints in route |

## Waypoints

### File format

Each route is a YAML file in `<routes_dir>/` (default: `<repo_root>/routes/`).
The filename stem must match the `route_name` field:

```yaml
route_name: my_route
description: "Optional description"
waypoints:
  - {x: 1.0, y: 2.0, yaw: 0.0}
  - {x: 2.5, y: 3.0, yaw: 1.57}
  - {x: 0.5, y: 1.0, yaw: -1.57}
```

- `x`, `y` — position in the `map` frame (metres)
- `yaw` — heading in radians

### Bundled example routes

| File | Waypoints | Description |
|---|---|---|
| `routes/demo_route.yaml` | 4 | Short triangle near spawn |
| `routes/perimeter_route.yaml` | 5 | Larger perimeter patrol |

### Custom routes_dir

Pass a different directory at launch time:

```bash
ros2 launch patrol_bringup patrol_system.launch.py routes_dir:=/path/to/my/routes
```

Or set the node parameter directly:

```bash
ros2 param set /patrol_executor routes_dir /path/to/my/routes
```

## Assumptions & Limitations

- **Map**: Pre-built map is included in `src/patrol_navigation/maps/`.
- **Localization**: Uses AMCL. If the estimated pose drifts, use **"2D Pose Estimate"** in RViz.
- **Display**: Requires an X11 server on the host. On headless servers, use a virtual display (e.g., `Xvfb`).
- **Robot model**: Defaults to TurtleBot3 Waffle. Change `TURTLEBOT3_MODEL` in `setup_env.bash` to use `burger`.

