# AR Robot Controller 🤖✋

ROS2 Humble Mobile Robot ကို **Hand Gesture** နဲ့ ထိန်းချုပ်တဲ့ **AR Web App** ဖြစ်ပါတယ်။

## Architecture

```
┌─────────────────────────────────────────┐
│  Phone / AR Glasses (WebXR Browser)     │
│  ┌───────────────┐  ┌───────────────┐   │
│  │  MediaPipe     │  │  Three.js      │  │
│  │  Hand Tracking │  │  AR Overlay    │  │
│  └───────┬───────┘  └───────┬───────┘   │
│          │    ┌─────────────┘            │
│  ┌───────▼────▼──────┐                  │
│  │    roslib.js       │                  │
│  │  (WebSocket client)│                  │
│  └───────┬────────────┘                  │
└──────────┼──────────────────────────────┘
           │ WebSocket (port 9090)
┌──────────▼──────────────────────────────┐
│  ROS2 Humble (Robot PC)                  │
│  ┌────────────────────┐                  │
│  │  rosbridge_server   │                  │
│  └────────┬───────────┘                  │
│  ┌────────▼───────────┐                  │
│  │ gesture_controller  │──► /cmd_vel     │
│  └────────────────────┘                  │
│  ┌────────────────────┐                  │
│  │  lidar_relay_node   │◄── /scan        │
│  └────────────────────┘                  │
└──────────────────────────────────────────┘
```

## Tech Stack

| Component | Technology |
|-----------|-----------|
| Hand Tracking | **MediaPipe Hands** (Google) |
| AR Rendering | **Three.js** + Canvas2D |
| ROS2 Bridge | **rosbridge_suite** (WebSocket) |
| Frontend | **Vanilla JS** + WebXR API |
| Robot Interface | **ROS2 Humble** (geometry_msgs/Twist) |
| LiDAR Viz | Canvas2D point cloud overlay |

## Hand Gestures

| Gesture | Action | Icon |
|---------|--------|------|
| ☝️ Index finger point | Forward | `INDEX_POINT` |
| ✌️ Two fingers (V) | Backward | `TWO_FINGERS` |
| ✊ Fist | Stop | `FIST` |
| 👈 Thumb left | Turn Left | `THUMB_LEFT` |
| 👉 Thumb right | Turn Right | `THUMB_RIGHT` |
| 🤏 Pinch | Speed Control | `PINCH` |
| 🖐️ Open hand | Emergency Stop | `OPEN_HAND` |

## Installation

### 1. Prerequisites

```bash
# ROS2 Humble installed
# Install rosbridge_suite
sudo apt install ros-humble-rosbridge-suite

# Install python dependencies
pip3 install mediapipe
```

### 2. Build the package

```bash
cd ~/Desktop/rom_ar_app/ros2_ws

# Build
colcon build --packages-select ar_robot_controller
source install/setup.bash
```

### 3. Make scripts executable

```bash
chmod +x src/ar_robot_controller/scripts/gesture_controller_node.py
chmod +x src/ar_robot_controller/scripts/lidar_relay_node.py
```

## Quick Start

### Terminal 1 — Launch AR Controller (ROS2 side)

```bash
cd ~/Desktop/rom_ar_app/ros2_ws
source install/setup.bash
ros2 launch ar_robot_controller ar_controller.launch.py
```

### Terminal 2 — Open Android App

Android app ကို ဖွင့်ပြီး Robot IP ထည့်ပါ။ Web app က Android app ထဲမှာ bundled ဖြစ်နေလို့ HTTP server မလိုပါ။

### Terminal 3 — (Optional) Robot Simulation

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

## ROS2 Topics

### Subscribed (from web app)
| Topic | Type | Description |
|-------|------|-------------|
| `/ar_controller/gesture` | `std_msgs/String` | JSON gesture data |
| `/ar_controller/joystick` | `std_msgs/Float32MultiArray` | [x, y] joystick values |

### Published (to web app & robot)
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/ar_controller/status` | `std_msgs/String` | JSON status feedback |
| `/ar_controller/lidar_viz` | `std_msgs/String` | Downsampled LiDAR for viz |

### Subscribed (from robot)
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |

## Configuration

Parameters can be set in [config/params.yaml](config/params.yaml) or via launch arguments:

```bash
ros2 launch ar_robot_controller ar_controller.launch.py \
    max_linear_vel:=0.3 \
    max_angular_vel:=0.8 \
    port:=9090
```

## Features

- ✅ **Hand Gesture Control** — MediaPipe hand tracking → ROS2 cmd_vel
- ✅ **Virtual Joystick** — Touch-based fallback control
- ✅ **LiDAR Visualization** — Real-time point cloud overlay on camera
- ✅ **Emergency Stop** — Open hand gesture or button
- ✅ **Velocity Smoothing** — Configurable acceleration limits
- ✅ **Watchdog Safety** — Auto-stop when connection lost
- ✅ **Dual Control Modes** — Switch between gesture and joystick
- ✅ **Mobile-First UI** — Optimized for phone / tablet / AR glasses

## Project Structure

```
ar_robot_controller/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── params.yaml              # Tunable parameters
├── launch/
│   └── ar_controller.launch.py  # Main launch (rosbridge + nodes)
├── scripts/
│   ├── gesture_controller_node.py  # Gesture → cmd_vel
│   └── lidar_relay_node.py         # LiDAR → web format
└── web_app/                         # Bundled in Android app assets
    ├── index.html               # Main AR interface
    └── js/
        ├── app.js               # App orchestrator
        ├── gesture_recognizer.js # MediaPipe hand tracking
        ├── ros_bridge.js        # roslib.js WebSocket
        ├── virtual_joystick.js  # Touch joystick
        └── lidar_visualizer.js  # LiDAR point cloud renderer
```

## Future Enhancements

- 🔲 WebXR Device API integration (immersive AR on supported glasses)
- 🔲 SLAM map overlay on camera feed
- 🔲 Navigation goal setting via AR markers
- 🔲 Multi-robot support
- 🔲 Voice commands integration
- 🔲 Robot camera feed overlay (compressed image topic)
