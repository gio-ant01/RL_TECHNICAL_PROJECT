# Escape Room - Robotics Lab 2025 Technical Project

Multi-robot cooperative system for autonomous navigation and task execution in a simulated corridor environment.


---

## 📋 Project Overview

This project implements an escape room scenario where two robots cooperate to navigate through a corridor and unlock an exit door:

- **fra2mo** (mobile robot): Autonomously navigates through the corridor, detects an ArUco marker on the exit door, and transmits its ID
- **armando** (robotic manipulator): Receives the marker ID and enters it as a PIN on a numeric keypad to unlock the door

### Key Features

✅ Custom Gazebo world with corridor, obstacles, and sliding door  
✅ Autonomous navigation with obstacle avoidance using Nav2  
✅ ArUco marker detection and recognition  
✅ 10 different manipulation controls (one per keypad digit)  
✅ Multi-robot communication via ROS2 topics  
✅ Coordinated task execution

---

## 🔨 Installation

### Prerequisites

```bash
# ROS 2 Humble
# Ubuntu 22.04
# Gazebo Ignition
```

### Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/escape_room_pkg.git
```

### Install Required Packages

```bash
sudo apt update

# Core ROS 2 packages
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-ros-gz-bridge
sudo apt install -y ros-humble-ros-gz-sim

# Navigation stack
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-slam-toolbox

# Controllers
sudo apt install -y ros-humble-controller-manager
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers

# ArUco detection
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y python3-opencv
```

### Clone ArUco ROS Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/pal-robotics/aruco_ros.git -b humble-devel
```

### Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## ✅ Usage

### **Complete Simulation (Recommended)**

This launches the entire escape room scenario with all components.

**Terminal 1** - Launch Gazebo world with fra2mo:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Spawn armando manipulator:
```bash
ros2 launch escape_room_pkg spawn_armando.launch.py
```

**Terminal 3** - Start complete mission (navigation + ArUco detection + PIN entry):
```bash
ros2 launch escape_room_pkg simulation.launch.py
```

The robot will:
1. ✅ Navigate autonomously to the door
2. ✅ Detect the ArUco marker
3. ✅ Send the ID to armando
4. ✅ Armando enters the PIN on the keypad
5. ✅ Door opens automatically
6. ✅ fra2mo can exit the corridor

---

## 🎮 Individual Components Testing

### **1. Test the Gazebo World Only**

```bash
ros2 launch escape_room_pkg gazebo_escape_room.launch.py
```

This launches only the corridor environment with obstacles and the exit door.

---

### **2. Manual Door Control**

**Terminal 1** - Launch simulation:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Open/close door manually:
```bash
# Open door
ros2 run escape_room_pkg door_controller.py

# Or publish directly
ros2 topic pub /door_slide/cmd_pos std_msgs/msg/Float64 "{data: 0.9}" --once
```

---

### **3. Armando Keypad Control**

Test armando's ability to press keypad buttons.

**Terminal 1** - Launch world + fra2mo:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Spawn armando:
```bash
ros2 launch escape_room_pkg spawn_armando.launch.py
```

**Terminal 3** - Start keypad controller:
```bash
ros2 run escape_room_pkg keypad_controller.py
```

**Terminal 4** - Send a PIN to test:
```bash
# Example: enter PIN "123"
ros2 topic pub /pin_code std_msgs/msg/String "{data: '123'}" --once

# Example: enter PIN "5678"
ros2 topic pub /pin_code std_msgs/msg/String "{data: '5678'}" --once
```

Armando will:
- Move to HOME position
- Press each digit sequentially via waypoints
- Return to HOME after each button press
- Open the door after complete PIN entry

#### Calibrated Button Positions

Each digit has a specific joint configuration `[j0, j1, j2, j3]`:

| Button | Joint Positions |
|--------|----------------|
| 0 | `[0.0, 0.5, 2.0, -0.5]` |
| 1 | `[0.32, 0.32, 0.62, 0.44]` |
| 2 | `[0.0, 0.28, 0.62, 0.52]` |
| 3 | `[-0.3, 0.32, 0.62, 0.44]` |
| 4 | `[0.32, 0.12, 1.38, 0.14]` |
| 5 | `[0.0, 0.0, 1.57, 0.0]` |
| 6 | `[-0.3, 0.12, 1.38, 0.14]` |
| 7 | `[0.32, 0.32, 1.88, -0.5]` |
| 8 | `[0.0, 0.26, 1.96, -0.58]` |
| 9 | `[-0.3, 0.32, 1.88, -0.5]` |
| HOME | `[0.0, -2.0, 1.5, 2.0]` |

---

### **4. Map Exploration and Saving**

Create a new map of the corridor environment.

**Terminal 1** - Launch simulation:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Start exploration:
```bash
ros2 launch escape_room_pkg fra2mo_explore.launch.py
```

fra2mo will autonomously explore the corridor and build the map.

**Terminal 3** - Save the map (while exploration is running):
```bash
cd ~/ros2_ws/src/escape_room_pkg
ros2 run nav2_map_server map_saver_cli -f maps/escape_room_map
```

This saves `escape_room_map.pgm` and `escape_room_map.yaml` in the `maps/` folder.

---

### **5. Autonomous Navigation (with existing map)**

**Terminal 1** - Launch simulation:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Launch navigation with saved map:
```bash
ros2 launch escape_room_pkg fra2mo_navigation.launch.py
```

This opens RViz2 with the pre-saved map. You can:
- Use **2D Pose Estimate** to set initial position
- Use **Nav2 Goal** to send navigation goals
- View real-time obstacle avoidance

#### Navigate to Goal via Script

**Terminal 3** - Execute waypoint navigation:
```bash
ros2 run escape_room_pkg follow_waypoints.py
```

Or single goal navigation:
```bash
ros2 run escape_room_pkg reach_goal.py
```

#### Find Goal Coordinates

To identify coordinates on the map:

```bash
ros2 topic echo /clicked_point --once
```

Then click **Publish Point** in RViz and click on the desired location.

---

### **6. ArUco Marker Detection**

Test camera-based marker recognition.

**Terminal 1** - Launch simulation:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Start ArUco detection:
```bash
ros2 launch aruco_ros marker_publisher.launch.py \
  marker_size:=0.06 \
  camera_image:=/fra2mo/camera \
  camera_info:=/fra2mo/camera_info \
  camera_frame:=camera_optical_link \
  reference_frame:=base_link
```

**Terminal 3** - View detected markers:
```bash
ros2 topic echo /marker_publisher/markers
```

Expected output when marker is detected:
```
markers:
- id: 376
  pose:
    position:
      x: -0.081
      y: -0.165
      z: 0.429
```

**Terminal 4** - Visualize detection:
```bash
ros2 run rqt_image_view rqt_image_view
```

Select topic: `/marker_publisher/result`

---

### **7. Complete ArUco → PIN Bridge**

Test the complete marker-to-PIN pipeline.

**Terminal 1** - Simulation:
```bash
ros2 launch escape_room_pkg fra2mo_escape_room.launch.py
```

**Terminal 2** - Spawn armando:
```bash
ros2 launch escape_room_pkg spawn_armando.launch.py
```

**Terminal 3** - Start keypad controller:
```bash
ros2 run escape_room_pkg keypad_controller.py
```

**Terminal 4** - Start ArUco detection:
```bash
ros2 launch aruco_ros marker_publisher.launch.py
```

**Terminal 5** - Start bridge node:
```bash
ros2 run escape_room_pkg aruco_to_pin_bridge.py
```

Position fra2mo in front of the door (manually in Gazebo or via navigation). Once the marker is detected for 5 consecutive frames, the PIN is automatically sent to armando.

---

## 📊 System Architecture

### ROS2 Nodes

| Node | Package | Function |
|------|---------|----------|
| `keypad_controller` | escape_room_pkg | Controls armando's button pressing |
| `door_controller` | escape_room_pkg | Opens/closes the exit door |
| `aruco_to_pin_bridge` | escape_room_pkg | Converts ArUco ID → PIN string |
| `marker_publisher` | aruco_ros | Detects ArUco markers from camera |
| `follow_waypoints` | escape_room_pkg | Guides fra2mo through waypoints |
| `nav2` | nav2_bringup | Autonomous navigation stack |

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/marker_publisher/markers` | `aruco_msgs/MarkerArray` | Detected ArUco markers |
| `/pin_code` | `std_msgs/String` | PIN sent to armando |
| `/pin_inserted` | `std_msgs/String` | Confirmation that PIN was entered |
| `/door_slide/cmd_pos` | `std_msgs/Float64` | Door position command |
| `/fra2mo/camera` | `sensor_msgs/Image` | Camera feed |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for fra2mo |
| `/arm/position_controller/commands` | `std_msgs/Float64MultiArray` | Joint commands for armando |

### Communication Flow

```
ArUco Marker (on door)
    ↓
fra2mo camera detects → /marker_publisher/markers
    ↓
aruco_to_pin_bridge converts ID → /pin_code
    ↓
keypad_controller moves armando → presses buttons
    ↓
keypad_controller confirms → /pin_inserted
    ↓
door_controller opens door → /door_slide/cmd_pos
    ↓
fra2mo exits corridor ✅
```

---

## 📁 Project Structure

```
escape_room_pkg/
├── launch/
│   ├── gazebo_escape_room.launch.py        # Gazebo world only
│   ├── fra2mo_escape_room.launch.py        # World + fra2mo spawn
│   ├── spawn_armando.launch.py             # Spawn armando
│   ├── fra2mo_explore.launch.py            # SLAM exploration
│   ├── fra2mo_navigation.launch.py         # Nav2 with map
│   ├── simulation.launch.py                # Complete mission
│   └── simulation2.launch.py               # Alternative (reach_goal)
├── worlds/
│   └── escape_room.sdf                     # Corridor environment
├── models/
│   ├── box_obstacle/                       # Corridor obstacles
│   ├── corridor_floor_*/                   # Floor segments
│   ├── corridor_wall_*/                    # Wall segments
│   ├── exit_door/                          # Sliding door with ArUco
│   │   ├── model.sdf
│   │   ├── model.config
│   │   └── aruco_tag.png                   # ArUco marker image
│   └── keypad/                             # Numeric keypad model
├── urdf/
│   ├── fra2mo/
│   │   ├── fra2mo.urdf.xacro              # Mobile robot description
│   │   ├── fra2mo_base_macro.xacro
│   │   ├── fra2mo_camera_macro.xacro      # Camera sensor
│   │   └── lidar_gazebo_macro.xacro       # LiDAR sensor
│   └── armando/
│       └── arm.urdf.xacro                  # Manipulator with pedestal
├── meshes/
│   ├── fra2mo/                             # Mobile robot meshes
│   └── armando/                            # Manipulator meshes
├── config/
│   ├── fra2mo/
│   │   ├── navigation.yaml                 # Nav2 parameters
│   │   └── exploration.yaml                # SLAM parameters
│   └── armando/
│       └── armando_controllers.yaml        # Joint controllers
├── maps/
│   ├── escape_room_map.pgm                 # Saved occupancy grid
│   └── escape_room_map.yaml                # Map metadata
├── scripts/
│   ├── door_controller.py                  # Door control node
│   ├── keypad_controller.py                # PIN entry controller
│   ├── aruco_to_pin_bridge.py             # ArUco → PIN converter
│   ├── follow_waypoints.py                 # Multi-waypoint navigation
│   └── reach_goal.py                       # Single goal navigation
├── src/
│   └── odom_bl_tf.cpp                      # Odometry TF publisher
├── rviz_conf/
│   └── navigation.rviz                     # RViz configuration
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Configuration

### Modify ArUco Marker ID

To change the exit code, replace the ArUco marker image:

1. Generate new marker at: https://chev.me/arucogen/
2. Save as `models/exit_door/aruco_tag.png`
3. Rebuild: `colcon build`

### Adjust Keypad Button Positions

Edit joint positions in `scripts/keypad_controller.py`:

```python
self.KEY_POSES = {
    '0': [0.0, 0.5, 2.0, -0.5],
    '1': [0.32, 0.32, 0.62, 0.44],
    # ... modify as needed
}
```

Test individual buttons:
```bash
ros2 topic pub /arm/position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.32, 0.32, 0.62, 0.44]}" --once
```

### Modify Navigation Waypoints

Edit coordinates in `scripts/follow_waypoints.py`:

```python
waypoints = yaml.safe_load('''
waypoints:
  - position: {x: 1.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: -0.707, w: 0.707}
  # Add more waypoints...
''')
```

---

## 🎯 Mission Parameters

### ArUco Detection Settings

| Parameter | Value | Description |
|-----------|-------|-------------|
| Marker size | 0.06 m | Physical size of printed marker |
| Consecutive frames | 5 | Frames required before sending PIN |
| Camera FOV | 80° | Horizontal field of view |
| Detection range | 0.05-15 m | Min/max detection distance |

### Door Control

| Parameter | Value | Description |
|-----------|-------|-------------|
| Closed position | 0.0 | Joint position when door is closed |
| Open position | 0.9 | Joint position when fully open |
| Open duration | 30 sec | Time door stays open after PIN entry |

### Navigation Tuning

Key parameters in `config/fra2mo/navigation.yaml`:

```yaml
local_costmap:
  robot_radius: 0.15        # Adjusted for fra2mo size
  inflation_radius: 0.25    # Obstacle inflation distance

global_costmap:
  resolution: 0.05          # Map resolution (m/cell)
```

---

## 🐛 Troubleshooting

### fra2mo doesn't move during navigation

```bash
# Check if Nav2 is active
ros2 topic echo /plan

# Verify costmaps are publishing
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap

# Check transform tree
ros2 run tf2_tools view_frames
```

### ArUco marker not detected

```bash
# Verify camera is publishing
ros2 topic hz /fra2mo/camera

# Check camera_info
ros2 topic echo /fra2mo/camera_info

# View raw camera feed
ros2 run rqt_image_view rqt_image_view /fra2mo/camera

# Ensure marker is in FOV and well-lit
```

### Armando doesn't press buttons correctly

```bash
# Check controller status
ros2 control list_controllers

# Verify joint states
ros2 topic echo /arm/joint_states

# Test manual position
ros2 topic pub /arm/position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0, -2.0, 1.5, 2.0]}" --once
```

### Door doesn't open after PIN entry

```bash
# Check if PIN was received
ros2 topic echo /pin_code

# Check if PIN confirmation was sent
ros2 topic echo /pin_inserted

# Manually trigger door
ros2 topic pub /door_slide/cmd_pos std_msgs/msg/Float64 "{data: 0.9}" --once
```

---

## 📝 Technical Details

### Robot Specifications

#### fra2mo (Mobile Robot)
- Type: Differential drive
- Sensors: 2D LiDAR, RGB camera
- Base dimensions: 0.35m × 0.25m × 0.08m
- Wheel radius: 0.03m
- Max velocity: 0.5 m/s

#### armando (Manipulator)
- Type: 4-DOF serial manipulator
- Pedestal height: 0.35m
- Joints: 4 revolute (`j0`, `j1`, `j2`, `j3`)
- Gripper: 2-finger parallel (not used in this project)
- Workspace: ~0.3m reach

### Simulation Environment

- Physics engine: ODE (Gazebo)
- Update rate: 1000 Hz
- Corridor dimensions: ~4m × 2m
- Obstacles: 5 boxes (various sizes)
- Exit door: Prismatic joint (0.9m travel)

---

## 👨‍💻 Author

**Giovanni Antonucci**  
Email: gio.antonucci@studenti.unina.it  
Robotics Lab 2025 - Università di Napoli Federico II  
Supervisor: Prof. Mario Selvaggio

---

## 📄 License

This project is developed for educational purposes as part of the Robotics Lab 2025 course.

---

## 🙏 Acknowledgments

- Prof. Mario Selvaggio for course supervision
- ROS 2 and Gazebo communities
- ArUco ROS package maintainers
- Nav2 navigation stack developers

---

**Last updated:** February 2026
