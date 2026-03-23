# FS Simulator

A full-stack **Formula Student Driverless** simulation environment built on **ROS2 Jazzy** and **Gazebo Harmonic**.

Generates FSG-spec race tracks, spawns a sensor-equipped formula car, and exposes everything over standard ROS2 topics — drop in any autonomous stack and go.

---

<!-- SCREENSHOT: TUI control panel (full terminal view) -->
> <img width="3179" height="1903" alt="image" src="https://github.com/user-attachments/assets/09f893fb-e067-4d58-93ed-fe586b144289" />


---

## Overview

<!-- SCREENSHOT or SHORT GIF: Gazebo simulation running with trackdrive layout, car driving -->
> ![2026-03-23 15-15-30](https://github.com/user-attachments/assets/7ff816a1-5b40-4ab8-85fd-405a5d1cbaec)


---

## Features

- **FSG-spec track types** — Trackdrive (~450 m), Test Track (~300 m), Skidpad (figure-8), Acceleration (75 m)
- **Custom map support** — drop any `.world` / `.sdf` file into `worlds/maps/` and it appears instantly in the TUI
- **Terminal control panel** — launch, stop, reset car, open RViz2 and PS teleop without leaving the terminal
- **Full sensor suite** — 32-ch VLP-16 LiDAR, ZED2i stereo camera, SBG IMU, SBG GPS
- **Split ROS–Gazebo bridge** — control and sensor topics on separate bridge nodes so LiDAR never blocks `cmd_vel`
- **Dynamic cones** — knocked cones react physically to the car (0.8 kg small / 1.5 kg large, Ackermann-correct friction)
- **PS4/PS5 controller teleop** — plug in and drive via `ros2 launch fs_vehicle ps_teleop.launch.py`

---

## Packages

| Package | Language | Purpose |
|---|---|---|
| `simulator` | Python | Track generator, TUI, bringup launch |
| `fs_vehicle` | C++ + Python | Car URDF, world SDF, launch, ROS–GZ bridge configs |
| `fs_vehicle_plugins` | C++ | Gazebo system plugin — custom Ackermann drive |

---

## Screenshots

### TUI Control Panel
<!-- SCREENSHOT: TUI with simulation running, log output visible -->
> <img width="3199" height="1905" alt="image" src="https://github.com/user-attachments/assets/c28de1af-df23-4095-97b3-a069a093deba" />


---

### Track Types

| Trackdrive (~450 m) | Test Track (~300 m) |
|---|---|
| <!-- SCREENSHOT: Trackdrive in Gazebo --> | <!-- SCREENSHOT: Test Track in Gazebo --> |
| <img width="3353" height="1769" alt="image" src="https://github.com/user-attachments/assets/c393c4b1-20bc-471f-b01f-6c4ba2661b4a" /> | <img width="3353" height="1769" alt="image" src="https://github.com/user-attachments/assets/67bbea31-f9a3-4e0b-8f6c-fed2456faec3" /> |

| Skidpad (figure-8) | Acceleration (75 m) |
|---|---|
| <!-- SCREENSHOT: Skidpad in Gazebo --> | <!-- SCREENSHOT: Acceleration in Gazebo --> |
| <img width="3353" height="1769" alt="image" src="https://github.com/user-attachments/assets/71b30713-c70c-47ce-ae51-cb076544e804" /> | <img width="3353" height="1769" alt="image" src="https://github.com/user-attachments/assets/6d6dc6eb-186c-4edf-b46b-d01a60236df0" /> |

---

### RViz2 — Sensor Visualisation
<!-- SCREENSHOT: RViz2 showing LiDAR point cloud, camera feed, TF tree -->
> 📸 *[Screenshot — RViz2 with LiDAR point cloud and camera]*

---

### Cone Physics
<!-- GIF or SHORT VIDEO: Car knocking cones over -->
> 🎬 *[GIF — car hitting cones]*

---

## Prerequisites

| Dependency | Version |
|---|---|
| Ubuntu | 24.04 |
| ROS2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8) |
| Python | 3.12 |
| `ros-jazzy-ros-gz-bridge` | latest |
| `ros-jazzy-robot-state-publisher` | latest |
| `ros-jazzy-rviz2` | latest |
| `python3-textual` | ≥ 0.50 |

Optional but recommended:
```bash
sudo apt install gamemode          # CPU governor boost during simulation
sudo apt install joystick          # PS4/PS5 teleop
```

---

## Installation

```bash
# 1. Clone into your ROS2 workspace
mkdir -p ~/Fs_ws/src && cd ~/Fs_ws/src
git clone https://github.com/your-username/fs-simulator.git .

# 2. Install ROS dependencies
cd ~/Fs_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --symlink-install

# 4. Source
source install/setup.bash
```

---

## Quick Start

### Terminal Control Panel (recommended)
```bash
ros2 run simulator fs_tui
```

<!-- SCREENSHOT: TUI on first launch (stopped state) -->
> 📸 *[Screenshot — TUI on first launch]*

### Direct launch
```bash
# Generate and launch a trackdrive
ros2 launch simulator sim.launch.py track_type:=trackdrive

# Load a custom world file
ros2 launch simulator sim.launch.py map_file:=/path/to/your.world

# Car only (static world)
ros2 launch fs_vehicle spawn_vehicle.launch.py
```

---

## Track Generator

Tracks are built from **hardcoded FSG circuit templates** using centripetal Catmull-Rom splines, then scaled to the target length and populated with cones at FSG-legal spacing.

```
Template points
    │
    ▼
Centripetal Catmull-Rom (α=0.5, 40 samples/segment)
    │
    ▼
Resample at 1.25 m spacing
    │
    ▼
Build boundaries (±1.5 m from centreline)
    │
    ▼
Place cones (5.0 m straights / 2.5 m corners)
    │
    ▼
Validate (turn radius ≥ 4 m, no overlaps, closed loop)
    │
    ▼
Write .world + _cones.yaml + _bridge.yaml + _meta.yaml
```

### Custom Maps

Drop any `.world` or `.sdf` file into:
```
simulator/fs_track/worlds/maps/
```
It will appear as a radio button in the TUI on next launch — no code changes needed.

---

## ROS2 Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | → Gazebo | Drive commands |
| `/odometry/wheels` | `nav_msgs/Odometry` | ← Gazebo | Wheel odometry |
| `/imu/data` | `sensor_msgs/Imu` | ← Gazebo | SBG IMU |
| `/scan` | `sensor_msgs/LaserScan` | ← Gazebo | LiDAR 2D scan |
| `/lidar/points` | `sensor_msgs/PointCloud2` | ← Gazebo | LiDAR point cloud |
| `/camera/image_raw` | `sensor_msgs/Image` | ← Gazebo | ZED2i left camera |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | ← Gazebo | Camera intrinsics |
| `/tf` | `tf2_msgs/TFMessage` | ← Gazebo | Transform tree |
| `/joint_states` | `sensor_msgs/JointState` | ← Gazebo | Wheel/steering joints |

---

## Architecture

```
User → TUI (panel.py) → SimController (sim_process.py) → ros2 launch
                                                               │
                                                       sim.launch.py
                                                          OpaqueFunction
                                                               │
                              ┌────────────────────────────────▼──────────────┐
                              │           TRACK GENERATOR PIPELINE             │
                              │   map_file set? ──YES──► Gazebo directly       │
                              │        │                                        │
                              │        NO                                       │
                              │        ▼                                        │
                              │   generate_versioned_track()                    │
                              │     layout.py  →  validation.py  →  world.py   │
                              │     artifacts.py  →  ros_bridge.py             │
                              └────────────────────────────────┬───────────────┘
                                                               │
                                                  spawn_vehicle.launch.py
                                                  ┌────────────▼──────────────┐
                                                  │   Gazebo (gz-sim 8)       │
                                                  │   DART physics @ 100 Hz   │
                                                  └──────────┬────────────────┘
                                                             │
                              ┌──────────────────────────────┼──────────────┐
                              │      ros_gz_bridge (×2)       │              │
                              │  control:  cmd_vel, odom, tf, imu, clock     │
                              │  sensors:  LiDAR, camera                     │
                              └─────────────────────────────────────────────┘
```

---

## Performance (RTF)

| Track | Cone count | RTF |
|---|---|---|
| Acceleration | ~36 | ~100% |
| Test Track | ~60–90 | ~90–95% |
| Trackdrive | ~170–200 | ~75–85% |
| Skidpad | ~188 | ~85–90% |

> RTF measured on an Intel Core i7-13700H, no discrete GPU involvement in physics.

---

## Teleop

### PlayStation Controller
```bash
ros2 launch fs_vehicle ps_teleop.launch.py
```
Or press **PS Teleop** in the TUI.

### Keyboard (manual ros2 run)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Project Structure

```
src/
├── simulator/
│   ├── fs_bringup/launch/          # sim.launch.py, view_generated_track.launch.py
│   ├── fs_track/
│   │   ├── meshes/                 # cone_blue.dae, cone_yellow.dae, cone_big.dae
│   │   └── worlds/
│   │       ├── generated/          # runtime-generated worlds (git-ignored)
│   │       └── maps/               # drop custom .world/.sdf files here
│   ├── fs_track_generator/
│   │   ├── models.py               # Cone, StartPose, TrackLayout dataclasses
│   │   ├── layout.py               # FSG template geometry + Catmull-Rom spline
│   │   ├── validation.py           # FSG rule checks
│   │   ├── world.py                # TrackLayout → Gazebo SDF
│   │   ├── artifacts.py            # versioned file I/O
│   │   ├── ros_bridge.py           # ROS–GZ bridge YAML generator
│   │   ├── pipeline.py             # generate → validate → write (top-level)
│   │   └── cli.py                  # `ros2 run simulator generate_track`
│   └── fs_tui/
│       ├── panel.py                # Textual TUI app
│       └── sim_process.py          # subprocess management
├── fs_vehicle/
│   ├── urdf/                       # fs_car.urdf.xacro + sensors
│   ├── meshes/                     # car body, wheels, sensor housings
│   ├── launch/                     # spawn_vehicle.launch.py, ps_teleop.launch.py
│   ├── config/                     # RViz2 config, bridge YAML
│   └── worlds/empty_track.sdf      # static world for car-only launch
├── fs_vehicle_plugins/
│   └── src/fs_ackermann_drive.cpp  # custom Gazebo Ackermann drive plugin
├── dashboard/PIPELINE.md           # deep-dive architecture reference
├── CHANGELOG.md                    # optimisation log
└── README.md
```

---

## Contributing

Pull requests are welcome. For major changes please open an issue first.

---

## License

MIT
