# Autonomous Color Detection & Navigation Robot (ROS 2 Humble & Gazebo)

An autonomous differential-drive navigation system in **ROS 2 Humble** and **Ignition Gazebo Fortress** powered by **OpenCV** real-time color detection from either the robot's simulated front camera or your laptop's physical webcam.

---

## Navigation Logic & Capabilities

1. **Red Object (`RED`)** $\to$ Navigates to the **Red Pillar** at `(2.5, 1.8)` and stops safely `0.35 m` (`0.75 m` center-to-center) in front of it.
2. **Green Object (`GREEN`)** $\to$ Navigates to the **Green Pillar** at `(3.5, 0.0)` (automatically bypassing an object placed in front of Home via `(1.0, 0.65)` when in Gazebo camera mode) and stops safely `0.35 m` in front of it.
3. **Any Other Color (`OTHER`)** (e.g., Blue, Yellow, Orange, Purple) $\to$ Navigates to the **White Pillar** at `(2.5, -1.8)` and stops safely `0.35 m` in front of it.
4. **Navigate From Anywhere** $\to$ Detecting a new color while parked at any pillar, mid-transit, or at Home redirects the robot directly to that color's pillar.
5. **Auto Object Cleanup & Exact Home Return** $\to$ Presented Gazebo test objects are removed once the robot detects and clears them before returning Home, and `home` docks cleanly at `(0.00, 0.00, Yaw=0.0°)` ready for the next run.

---

## Workspace Structure

```text
color_nav_ws/
├── .gitignore
├── README.md
└── src/
    ├── color_navigation/
    │   ├── color_navigation/
    │   │   ├── color_detector.py         # Gazebo camera HSV color detector
    │   │   ├── webcam_color_detector.py  # Laptop webcam OpenCV color detector + hotkeys
    │   │   ├── vision_utils.py           # Shared ROS Image conversion & HSV masking
    │   │   ├── navigator.py              # Closed-loop controller & pillar-to-pillar FSM
    │   │   ├── gazebo_utils.py           # Safe Gazebo scene/info & entity removal helper
    │   │   ├── spawn_object.py           # CLI tool to spawn/delete Gazebo test boxes
    │   │   ├── return_home.py            # CLI tool (`home` / `return_home`)
    │   │   └── demo_runner.py            # Automated 3-color mission demo script
    │   ├── launch/
    │   │   └── autonomous_nav.launch.py  # Main simulation + navigation + RViz2 launch
    │   ├── models/                       # red_box, green_box, blue_box, yellow_box SDFs
    │   └── worlds/                       # empty_arena, world_red, world_green, world_other
    └── robot_description/
        ├── config/view_robot.rviz        # Pre-configured RViz2 layout
        ├── launch/
        │   ├── display.launch.py         # URDF + Joint State Publisher + RViz2 viewer
        │   └── gazebo.launch.py          # Standalone Gazebo robot spawn launch
        ├── urdf/differential_robot.urdf.xacro
        └── worlds/color_world.sdf
```

---

## Arena Coordinates

| Entity | $X$ | $Y$ | $Z$ | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Home Docking Pad** | `0.00 m` | `0.00 m` | `0.001 m` | Robot origin & `Yaw = 0.0°` |
| **Presentation Zone** | `0.90 m` | `0.00 m` | `0.175 m` | Gazebo test object spawn location |
| **Red Pillar** | `2.50 m` | `+1.80 m` | `0.60 m` | Radius `0.20 m`, target ring `0.45 m` |
| **Green Pillar** | `3.50 m` | `0.00 m` | `0.60 m` | Radius `0.20 m`, target ring `0.45 m` |
| **White Pillar** | `2.50 m` | `-1.80 m` | `0.60 m` | Radius `0.20 m`, target ring `0.45 m` |

---

## Build Instructions

```bash
cd /home/user/color_nav_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

### 1. Laptop Webcam Mode (Physical Camera -> Gazebo Robot)
Hold real colored objects in front of your laptop's webcam to drive the robot to the corresponding pillar from anywhere:

```bash
ros2 launch color_navigation autonomous_nav.launch.py webcam:=true rviz:=true
```
* Optional camera index override (e.g., `/dev/video2`): add `webcam_device:=2`.
* Webcam preview hotkeys: **`H`** = Return Home, **`R`** = Reset lock, **`Q`** = Quit.
* You can also run `ros2 run color_navigation webcam` in a separate terminal alongside `manual:=true`; Gazebo's onboard camera detector yields automatically while the webcam node is active.

### 2. Manual Gazebo Object Mode
```bash
# Terminal 1: Launch empty arena with pillars, robot, detector, navigator & RViz2
ros2 launch color_navigation autonomous_nav.launch.py manual:=true rviz:=true

# Terminal 2: Present or remove objects manually and command Home
ros2 run color_navigation spawn_object --color red
ros2 run color_navigation spawn_object --color green
ros2 run color_navigation spawn_object --color blue     # OTHER -> White Pillar
ros2 run color_navigation spawn_object --delete         # Optional manual delete
ros2 run color_navigation home                          # Return to exact Home (0, 0, 0°)
```

### 3. Auto-Spawn Gazebo Mode
```bash
ros2 launch color_navigation autonomous_nav.launch.py color:=red rviz:=true
# Supports color:=red | green | other
```

### 4. Automated 3-Cycle Demo & URDF Viewer
```bash
# Sequential Red -> Home -> Green -> Home -> Blue -> Home test runner:
ros2 run color_navigation demo

# Inspect robot URDF and wheel joints standalone in RViz2:
ros2 launch robot_description display.launch.py
```

---

## ROS 2 Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/detected_color` | `std_msgs/msg/String` | Confirmed color (`"RED"`, `"GREEN"`, `"OTHER"`) |
| `/camera/image` | `sensor_msgs/msg/Image` | Simulated front camera stream |
| `/camera/debug_image` | `sensor_msgs/msg/Image` | Annotated detection overlay shown in RViz2 |
| `/webcam/debug_image` | `sensor_msgs/msg/Image` | Annotated laptop webcam feed (standalone mode) |
| `/webcam_active` | `std_msgs/msg/Empty` | Signals webcam priority over Gazebo camera |
| `/planned_path` | `nav_msgs/msg/Path` | Active navigation trajectory for RViz2 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Differential drive linear & angular velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Ground-truth Gazebo pose feedback |
| `/return_home` | `std_msgs/msg/Empty` | Commands robot back to `(0.00, 0.00, 0.0°)` |
| `/reset_detector` | `std_msgs/msg/Empty` | Resets detector lock & starts stabilization window |
