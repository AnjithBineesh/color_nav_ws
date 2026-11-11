# Autonomous Color Detection & Navigation Robot (ROS 2 Humble & Gazebo)

An end-to-end **ROS 2 Humble** robotics project that combines **real-time OpenCV computer vision** with **closed-loop autonomous mobile robot navigation** in **Ignition Gazebo Fortress**.

A 4-wheel differential-drive robot inspects the color of an object presented to it — either via the **simulated front camera in Gazebo** or via your **laptop's physical webcam** — and autonomously plans a path to the matching color-coded pillar in the arena while stopping at a safe distance without colliding.

---

## How It Works

```text
  ┌─────────────────────────┐      /detected_color       ┌─────────────────────────────┐
  │   OpenCV Vision Node    │  ("RED" | "GREEN" | "OTHER")│   PillarNavigator FSM       │
  │  • Gazebo Front Camera  ├────────────────────────────▶│  • 20 Hz P-controller       │
  │  • OR Laptop Webcam     │                             │  • Safe 0.35 m pillar stop  │
  └─────────────────────────┘                             │  • Auto object cleanup      │
               ▲                                          └──────────────┬──────────────┘
               │ /camera/image                                           │ /cmd_vel
               │                                                         ▼
  ┌────────────┴─────────────────────────────────────────────────────────┴──────────────┐
  │                       Ignition Gazebo Fortress + ros_gz_bridge                       │
  │  • DiffDrive Physics   • Ground-Truth /odom & /tf   • Red / Green / White Pillars   │
  └─────────────────────────────────────────────────────────────────────────────────────┘
```

### Key Features & Navigation Rules

| Color Detected | Destination Pillar | Arena Coordinates | Behavior |
| :--- | :--- | :--- | :--- |
| **`RED`** | Red Pillar | `(2.5, 1.8)` | Turns left, drives directly, stops `0.35 m` (`0.75 m` center) in front |
| **`GREEN`** | Green Pillar | `(3.5, 0.0)` | Bypasses presented Gazebo box via `(1.0, 0.65)` when near Home, stops `0.35 m` in front |
| **`OTHER`** (Blue / Yellow / etc.) | White Pillar | `(2.5, -1.8)` | Turns right, drives directly, stops `0.35 m` in front |
| **`home` Command** | Home Docking Pad | `(0.00, 0.00, 0.0°)` | Returns accurately (`< 4 cm`, `< 2.3°`) and resets detectors to standby |

* **Switch Colors From Anywhere**: Presenting a new color redirects the robot directly to that color's pillar whether it is at Home, parked at another pillar, or mid-navigation.
* **Foreground vs. Background Filtering**: Size, pixel area, and ground-baseline thresholds reject distant pillars so Gazebo's camera only triggers on an object presented in front of the robot.
* **No Infinite Re-Detection Loops**: Presented Gazebo boxes are automatically deleted from the simulation once cleared after detection so the robot docks cleanly when returning Home.

---

## Prerequisites & Dependencies

### System Requirements
* **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
* **Middleware**: ROS 2 Humble Hawksbill
* **Simulator**: Ignition Gazebo Fortress (`ign-gazebo6`)
* **Python**: Python 3.10+
* **Hardware (Optional)**: Built-in or USB webcam (`/dev/video0`, `/dev/video1`, ...) for physical webcam mode

### Required ROS 2 & System Packages

Install all required ROS 2 simulation, visualization, URDF, and OpenCV dependencies:

```bash
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-cv-bridge \
  python3-opencv \
  python3-numpy \
  python3-colcon-common-extensions
```

#### Package Roles
* `ros-humble-ros-gz-sim` & `ros-humble-ros-gz-bridge`: Runs Ignition Gazebo worlds, spawns models, and bridges `/clock`, `/cmd_vel`, `/odom`, `/joint_states`, `/tf`, `/camera/image`, and `/camera/camera_info`.
* `ros-humble-xacro`, `ros-humble-robot-state-publisher`, `ros-humble-joint-state-publisher(-gui)`: Parses `differential_robot.urdf.xacro` and publishes TF/joint states.
* `ros-humble-rviz2`: Visualizes TF frames, camera streams, `/camera/debug_image`, and `/planned_path`.
* `python3-opencv`, `python3-numpy`, `ros-humble-cv-bridge`: HSV color thresholding, morphological noise filtering, webcam capture, and ROS `Image` conversion (with a built-in NumPy fallback if `cv_bridge` is absent).

> **Webcam Permission Note**: If `/dev/video0` fails to open in webcam mode, ensure your user belongs to the `video` group (`sudo usermod -aG video $USER`, then log out and back in) and close any other app using the camera.

---

## Installation & Build

```bash
# 1. Clone the repository into a ROS 2 workspace (or enter existing workspace)
cd /home/user/color_nav_ws

# 2. Source ROS 2 Humble and build all packages
source /opt/ros/humble/setup.bash
colcon build

# 3. Source the built workspace (run in every new terminal)
source /home/user/color_nav_ws/install/setup.bash
```

---

## Launch Commands

### 1. Full Navigation Simulation (`color_navigation`)

```bash
# Laptop Webcam + Gazebo + RViz2 (hold real objects in front of your webcam)
ros2 launch color_navigation autonomous_nav.launch.py webcam:=true rviz:=true

# Laptop Webcam using a secondary camera device (e.g., /dev/video2)
ros2 launch color_navigation autonomous_nav.launch.py webcam:=true webcam_device:=2 rviz:=true

# Manual Gazebo Object Mode (empty arena; spawn boxes or launch webcam separately)
ros2 launch color_navigation autonomous_nav.launch.py manual:=true rviz:=true

# Auto-Spawn Gazebo Object Mode (color:=red | green | other)
ros2 launch color_navigation autonomous_nav.launch.py color:=red rviz:=true
ros2 launch color_navigation autonomous_nav.launch.py color:=green rviz:=true
ros2 launch color_navigation autonomous_nav.launch.py color:=other rviz:=true

# Headless Gazebo Server Only (no Gazebo GUI window)
ros2 launch color_navigation autonomous_nav.launch.py manual:=true headless:=true
```

#### `autonomous_nav.launch.py` Arguments

| Argument | Default | Options / Description |
| :--- | :--- | :--- |
| `webcam` | `false` | `true` \| `false` — Use laptop webcam instead of Gazebo camera |
| `webcam_device` | `0` | Video device index (`/dev/video<N>`) |
| `manual` | `false` | `true` \| `false` — Start empty arena (`empty_arena.sdf`) without pre-spawned box |
| `color` | `red` | `red` \| `green` \| `other` — Auto-spawned box color when `manual:=false` & `webcam:=false` |
| `rviz` | `false` | `true` \| `false` — Launch RViz2 with `view_robot.rviz` |
| `headless` | `false` | `true` \| `false` — Run Ignition Gazebo server-only (`-r -s`) |

---

### 2. Robot URDF & Standalone Gazebo (`robot_description`)

```bash
# View robot URDF model in RViz2 with interactive wheel joint sliders
ros2 launch robot_description display.launch.py

# View robot URDF in RViz2 without the joint GUI window
ros2 launch robot_description display.launch.py use_gui:=false

# View robot URDF synchronized with simulation clock
ros2 launch robot_description display.launch.py use_sim_time:=true

# Spawn robot standalone into Gazebo (without navigation/vision nodes)
ros2 launch robot_description gazebo.launch.py

# Standalone Gazebo robot spawn + RViz2
ros2 launch robot_description gazebo.launch.py rviz:=true

# Standalone Gazebo spawn with a custom SDF world path
ros2 launch robot_description gazebo.launch.py world:=/home/user/color_nav_ws/src/color_navigation/worlds/empty_arena.sdf rviz:=true
```

#### `display.launch.py` & `gazebo.launch.py` Arguments

| Launch File | Argument | Default | Description |
| :--- | :--- | :--- | :--- |
| `display.launch.py` | `use_gui` | `true` | Enable `joint_state_publisher_gui` sliders |
| `display.launch.py` | `use_sim_time` | `false` | Use `/clock` simulation time |
| `display.launch.py` | `rviz_config` | `view_robot.rviz` | Path to RViz2 configuration file |
| `gazebo.launch.py` | `world` | `color_world.sdf` | Path to SDF world file |
| `gazebo.launch.py` | `rviz` | `false` | Launch RViz2 alongside Gazebo |

---

## CLI Node & Control Commands (`ros2 run`)

### Laptop Webcam Detector

Run standalone alongside `manual:=true`:

```bash
# Default camera (/dev/video0, auto-falls back across 0..3)
ros2 run color_navigation webcam
# Alias:
ros2 run color_navigation webcam_detector

# Override camera index, HSV saturation sensitivity, or headless preview
ros2 run color_navigation webcam --ros-args -p device_id:=2 -p show_window:=true -p min_fill_ratio:=0.15
```

* **Webcam Window Hotkeys**: **`H`** Return Home · **`R`** Reset detector lock · **`Q`** / **`ESC`** Quit

### Gazebo Test Object Spawner

```bash
# Present colored boxes at (0.9, 0.0, 0.175) in front of the robot
ros2 run color_navigation spawn_object --color red
ros2 run color_navigation spawn_object --color green
ros2 run color_navigation spawn_object --color blue     # Any non-Red/Green -> White Pillar
ros2 run color_navigation spawn_object --color yellow   # Any non-Red/Green -> White Pillar

# Custom pose or model name
ros2 run color_navigation spawn_object --color red --x 0.9 --y 0.0 --z 0.175 --name presented_object

# Delete spawned box or return Home directly from spawner CLI
ros2 run color_navigation spawn_object --delete
ros2 run color_navigation spawn_object --home
```

### Home Return, Automated Demo & Standalone Nodes

```bash
# Send robot back to exact Home (0.00, 0.00, 0.0°) and reset detector
ros2 run color_navigation home
# Alias:
ros2 run color_navigation return_home

# Run automated 3-cycle mission test (Red -> Home -> Green -> Home -> Blue -> Home)
ros2 run color_navigation demo

# Run individual ROS 2 nodes directly if needed
ros2 run color_navigation color_detector
ros2 run color_navigation navigator --ros-args -p world_name:=empty_arena
```

### Quick Topic Debugging Commands

```bash
# Monitor detected color stream
ros2 topic echo /detected_color

# Manually command a pillar color without camera/spawner
ros2 topic pub --once /detected_color std_msgs/msg/String "{data: 'RED'}"

# Manually trigger Return Home via topic
ros2 topic pub --once /return_home std_msgs/msg/Empty "{}"

# Inspect robot pose & velocity commands
ros2 topic echo /odom
ros2 topic echo /cmd_vel
```

---

## Workspace Layout & ROS 2 Topics

```text
color_nav_ws/
├── .gitignore
├── README.md
└── src/
    ├── color_navigation/
    │   ├── color_navigation/
    │   │   ├── color_detector.py         # Gazebo front-camera HSV detector
    │   │   ├── webcam_color_detector.py  # Laptop webcam HSV detector + window hotkeys
    │   │   ├── vision_utils.py           # Shared ROS Image conversion & HSV masking
    │   │   ├── navigator.py              # Closed-loop controller & pillar-to-pillar FSM
    │   │   ├── gazebo_utils.py           # Safe scene/info query & entity cleanup
    │   │   ├── spawn_object.py           # Gazebo test box spawner/deleter CLI
    │   │   ├── return_home.py            # Home return CLI (`home` / `return_home`)
    │   │   └── demo_runner.py            # Automated 3-color cycle runner
    │   ├── launch/autonomous_nav.launch.py
    │   ├── models/                       # red_box, green_box, blue_box, yellow_box SDFs
    │   └── worlds/                       # empty_arena, world_red, world_green, world_other
    └── robot_description/
        ├── config/view_robot.rviz
        ├── launch/{display,gazebo}.launch.py
        ├── urdf/differential_robot.urdf.xacro
        └── worlds/color_world.sdf
```

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/detected_color` | `std_msgs/msg/String` | Confirmed color (`"RED"`, `"GREEN"`, `"OTHER"`) |
| `/camera/image` | `sensor_msgs/msg/Image` | Simulated robot camera stream |
| `/camera/debug_image` | `sensor_msgs/msg/Image` | Annotated detection overlay shown in RViz2 |
| `/webcam/debug_image` | `sensor_msgs/msg/Image` | Annotated laptop webcam stream (standalone mode) |
| `/webcam_active` | `std_msgs/msg/Empty` | Gives laptop webcam priority over Gazebo detector |
| `/planned_path` | `nav_msgs/msg/Path` | Active planned trajectory visualized in RViz2 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Differential-drive velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Ground-truth Gazebo pose feedback |
| `/return_home` | `std_msgs/msg/Empty` | Sends robot back to `(0.00, 0.00, 0.0°)` |
| `/reset_detector` | `std_msgs/msg/Empty` | Resets detector lock and starts stabilization cooldown |
