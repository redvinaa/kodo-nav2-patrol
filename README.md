# 🧭 Kodo Nav2 Patrol System

## Project Overview
A modular **ROS 2 + Nav2 based patrol system** designed for simulation-first development of autonomous monitoring and inspection workflows.

This project focuses on building a **practical patrol software layer** that can later be extended to real robots (example, quadrupeds like Unitree Go2).

**Key Capabilities (MVP Focus):**
- Autonomous waypoint-based patrol
- Patrol lifecycle control (start/pause/resume/stop)
- Basic geofence enforcement
- Simulation-based testing (Gazebo)
- Extensible architecture for monitoring, vision, and web control

## 🚀 Features (MVP → Advanced)

### 1. Patrol Route Execution
- **Waypoint Patrol:** Define patrol routes using waypoint sequences
- **Autonomous Navigation:** Execute patrols using Nav2
- **Looping Patrol:** Repeat routes continuously
- **Patrol Control:** Start, pause, resume, and stop patrol execution

### 2. Geofencing (Basic)
- **Zone Definition:** Define keep-in / keep-out zones (config-based)
- **Boundary Monitoring:** Continuously track robot position
- **Breach Handling:**
  - Stop robot on violation
  - Log alert event

### 3. Route Management (Planned)
- Save/load patrol routes
- Teach & repeat (teleop recording → waypoint extraction)

### 4. Monitoring Layer (Planned)
- Web-based dashboard
- Robot state visualization
- Patrol control interface

### 5. Vision & Detection (Planned)
- Camera streaming
- Person detection (YOLO or similar)
- Event-based alerts

### 6. Advanced Features (Future)
- Autonomous charging & docking
- Multi-robot patrol coordination
- Cloud-based monitoring

## ⚙️ Technical Stack

| Layer | Technology |
|------|------------|
| Middleware | ROS 2 (Humble) |
| Navigation | Nav2 |
| Simulation | Gazebo |
| Visualization | RViz |
| Mapping | SLAM Toolbox / AMCL |
| Backend (Planned) | FastAPI / Node.js |
| Frontend (Planned) | React / Vue |
| Communication | rosbridge / WebSockets |

## 🏗️ System Architecture (Core)

```
[patrol_manager]
|
v
[NavigateToPose / Nav2]
|
v
[robot_base]

[geofence_monitor] —> /cmd_vel stop on violation
```

## 🧩 Development Roadmap

### 🔹 Phase 0 — Foundation
- [ ] Gazebo simulation setup
- [ ] Robot + map + localization
- [ ] Nav2 goal navigation working

### 🔹 Phase 1 — Patrol Core (MVP)
- [ ] Waypoint patrol execution
- [ ] Looping patrol
- [ ] Pause / resume / stop
- [ ] Basic geofencing

### 🔹 Phase 2 — Route Management
- [ ] Route save/load
- [ ] Teach & repeat mode

### 🔹 Phase 3 — Monitoring
- [ ] Backend API
- [ ] Minimal web UI
- [ ] Robot state visualization

### 🔹 Phase 4 — Vision
- [ ] Camera streaming
- [ ] Object detection

### 🔹 Phase 5 — Advanced
- [ ] Autonomous charging
- [ ] Docking
- [ ] Deterrence features

## 🛠️ Getting Started

### 1. Clone Repository
```bash
git clone https://github.com/Kodo-Robotics/kodo-nav2-patrol
cd kodo-nav2-patrol
```

2. Install Dependencies
```bash
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
```

3. Launch Simulation
```bash
ros2 launch patrol_bringup sim_launch.py
```

4. Run Navigation
```bash
ros2 launch patrol_bringup nav_launch.py
```

## 📄 License

Licensed under the Apache 2.0 License.
