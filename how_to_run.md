# How to Run — AR Robot Controller 🤖✋

ROS2 Humble Mobile Robot ကို လက်ချောင်းတွေနဲ့ ထိန်းချုပ်တဲ့ AR App ကို Run ဖို့ အဆင့်ဆင့် ရှင်းပြပေးပါမယ်။

---

## 1. Prerequisites (ကြိုတင်ပြင်ဆင်ရမယ့်အရာများ)

### 1.1 ROS2 Humble

```bash
# ROS2 Humble install ထားပြီးသားဖြစ်ရပါမယ်
# https://docs.ros.org/en/humble/Installation.html

# ROS2 environment source လုပ်ပါ
source /opt/ros/humble/setup.bash
```

### 1.2 rosbridge_suite Install

```bash
sudo apt update
sudo apt install ros-humble-rosbridge-suite
```

### 1.3 Python Dependencies

```bash
pip3 install mediapipe
```

> **Note**: `mediapipe` က Robot PC ဘက်မှာ install ဖို့ မလိုပါ။ Browser ထဲမှာ CDN ကနေ load လုပ်ပါတယ်။
> `rclpy`, `geometry_msgs`, `sensor_msgs` တွေက ROS2 Humble နဲ့ ပါပြီးသားပါ။

---

## 2. Package Build လုပ်ခြင်း

```bash
# Workspace directory ကို သွားပါ
cd ~/Desktop/rom_ar_app/ros2_ws

# Package ကို build လုပ်ပါ
colcon build --packages-select ar_robot_controller

# Build ပြီးရင် source လုပ်ပါ
source install/setup.bash
```

### Scripts ကို executable ပြောင်းပါ (ပထမဆုံးတစ်ကြိမ်ပဲ)

```bash
chmod +x src/ar_robot_controller/scripts/gesture_controller_node.py
chmod +x src/ar_robot_controller/scripts/lidar_relay_node.py
```

---

## 3. Run ပုံ (Terminal ၁ ခု လိုပါမယ်)

### Terminal 1 — AR Controller Launch (ROS2 nodes + rosbridge)

```bash
cd ~/Desktop/rom_ar_app/ros2_ws
source install/setup.bash

# Default settings နဲ့ launch
ros2 launch ar_robot_controller ar_controller.launch.py
```

ဒီ command က အောက်ပါ ၃ ခုကို တစ်ခါတည်း ဖွင့်ပေးပါတယ်:
- **rosbridge_websocket** (port 9090) — ဖုန်းနဲ့ ROS2 ချိတ်ဆက်ပေးတဲ့ WebSocket server
- **gesture_controller_node** — Gesture data ကနေ `/cmd_vel` ပြောင်းပေးတဲ့ node
- **lidar_relay_node** — `/scan` ကနေ web-friendly JSON ပြောင်းပေးတဲ့ node

#### Custom parameters နဲ့ launch ချင်ရင်:

```bash
ros2 launch ar_robot_controller ar_controller.launch.py \
    max_linear_vel:=0.3 \
    max_angular_vel:=0.8 \
    port:=9090
```

### Terminal 2 — (Optional) Robot Simulation ဖွင့်ခြင်း

```bash
# TurtleBot3 simulation ကို test အတွက် သုံးလို့ရပါတယ်
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 4. Android App ဖွင့်ခြင်း

Web app က Android app ထဲမှာ bundled ဖြစ်နေလို့ HTTP server မလိုပါ။

### 4.1 Robot PC ရဲ့ IP ကို ရှာပါ

```bash
# Robot PC မှာ
hostname -I
# ဥပမာ: 192.168.1.100
```

### 4.2 Android App ဖွင့်ပါ

1. **ဖုန်းမှာ App ဖွင့်ပါ** → "AR Robot Controller" icon
2. **Robot IP ထည့်ပါ** → Connection Settings မှာ
3. **WebSocket Port** → 9090 (ပုံမှန်)
4. **Test Connection** → Robot PC reachable ဖြစ်/ မဖြစ် စစ်ပါ
5. **Save & Connect** → AR controller ကို auto-load လုပ်ပါမယ်

### 4.3 Camera Permission

ဖုန်းက Camera access တောင်းလာရင် **Allow** ပေးပါ။ Hand tracking အတွက် camera လိုပါတယ်။

> ⚠️ **အရေးကြီး**: ဖုန်းနဲ့ Robot PC က **တူညီတဲ့ WiFi network** ထဲမှာ ရှိရပါမယ်!

---

## 5. App သုံးပုံ

### 5.1 Control Modes

App ဖွင့်လိုက်ရင် ညာဘက်အပေါ်မှာ ၂ ခု ရွေးလို့ရပါတယ်:

| Mode | Description |
|------|-------------|
| **✋ Gesture** | လက်ဟန်ခြေဟန်နဲ့ control (default) |
| **🕹️ Joystick** | Screen ပေါ်က virtual joystick နဲ့ control |

### 5.2 Hand Gestures

Camera ရှေ့မှာ လက်ကို ပြပါ:

| Gesture | လက်ပုံ | Robot Action |
|---------|--------|-------------|
| ☝️ **Index Point** | လက်ညှိုးတစ်ချောင်းထောင် | ရှေ့တိုး |
| ✌️ **Two Fingers** | လက်ညှိုး + လက်ခလယ် | နောက်ဆုတ် |
| ✊ **Fist** | လက်ဆုပ် | ရပ် |
| 👈 **Thumb Left** | လက်မ ဘယ်ညွှန် | ဘယ်ကွေ့ |
| 👉 **Thumb Right** | လက်မ ညာညွှန် | ညာကွေ့ |
| 🤏 **Pinch** | လက်မနဲ့လက်ညှိုးကပ် | Speed ပြောင်း |
| 🖐️ **Open Hand** | လက်ဖဝါးပြ | Emergency Stop |

### 5.3 UI Elements

- **Status Bar** (အပေါ်ဆုံး) — ROS connection status + Hand detection status
- **Gesture Display** — လက်ရှိ gesture ပြ
- **Velocity Display** (ဘယ်ဘက်အောက်) — Linear/Angular velocity ပြ
- **E-STOP Button** (အလယ်အောက်) — Emergency stop / Resume ခလုတ်
- **LiDAR Overlay** — Robot ပတ်ဝန်းကျင်က obstacle တွေကို အရောင်နဲ့ ပြ

### 5.4 Settings (⚙️ ခလုတ်)

- **ROS Bridge URL** — rosbridge WebSocket URL ပြောင်း
- **Max Speed** — အမြင့်ဆုံး velocity slider
- **Show LiDAR** — LiDAR overlay ဖွင့်/ပိတ်
- **Show Hand** — Hand landmark overlay ဖွင့်/ပိတ်

---

## 6. ROS2 Topics စစ်ဆေးခြင်း

App run နေစဉ် topics တွေကို စစ်ကြည့်လို့ရပါတယ်:

```bash
# Active topics list
ros2 topic list

# cmd_vel output ကြည့်ခြင်း
ros2 topic echo /cmd_vel

# Gesture data ကြည့်ခြင်း
ros2 topic echo /ar_controller/gesture

# Status feedback ကြည့်ခြင်း
ros2 topic echo /ar_controller/status

# Topic frequency စစ်ခြင်း
ros2 topic hz /cmd_vel
```

---

## 7. Troubleshooting

### ❌ ဖုန်းက ROS connect မရဘူး

```bash
# rosbridge run နေသလား စစ်ပါ
ros2 topic list | grep rosbridge

# Firewall စစ်ပါ
sudo ufw allow 9090

# WebSocket port test
# Android app မှာ Connection Settings က Robot IP မှန်သလား စစ်ပါ
```

### ❌ Camera ဖွင့်လို့မရဘူး

- HTTPS setup ပြီးပြီလား စစ်ပါ (Section 4.4)
- Browser permission allow ပေးပြီးပြီလား စစ်ပါ
- Chrome/Firefox သုံးပါ (Safari limited support)

### ❌ Hand tracking မလုပ်ဘူး

- Camera ကို လက်ကောင်းကောင်းပြပါ
- အလင်းရောင်ကောင်းတဲ့နေရာမှာ သုံးပါ
- MediaPipe CDN ကို internet connection လိုပါတယ် (ပထမတစ်ကြိမ်)

### ❌ Robot မရွေ့ဘူး

```bash
# cmd_vel data ထွက်နေသလား စစ်ပါ
ros2 topic echo /cmd_vel

# Emergency stop ဖြစ်နေသလား ဖုန်းမှာ ကြည့်ပါ
# E-STOP button အစိမ်းဖြစ်နေရင် RESUME နှိပ်ပါ

# Robot driver run နေသလား စစ်ပါ
ros2 node list
```

### ❌ LiDAR မပြဘူး

```bash
# /scan topic ရှိသလား စစ်ပါ
ros2 topic echo /scan --once

# lidar_relay_node run နေသလား
ros2 node list | grep lidar
```

---

## 8. Quick Reference

```bash
# ============ တစ်ခါတည်း Run ပုံ ============

# Terminal 1: ROS2 nodes
cd ~/Desktop/rom_ar_app/ros2_ws && source install/setup.bash
ros2 launch ar_robot_controller ar_controller.launch.py

# Android App: Install APK → Open → Enter Robot IP → Connect
```

---

## 9. ဖိုင်တွေ ဘာလုပ်လဲ

| File | Description |
|------|-------------|
| `scripts/gesture_controller_node.py` | Gesture data → `/cmd_vel` Twist message ပြောင်းပေးတဲ့ ROS2 node |
| `scripts/lidar_relay_node.py` | `/scan` LaserScan → web-friendly JSON ပြောင်းပေးတဲ့ node |
| `web_app/index.html` | AR UI layout (camera feed + overlays) |
| `web_app/js/gesture_recognizer.js` | MediaPipe Hands → gesture classification |
| `web_app/js/ros_bridge.js` | roslib.js WebSocket → ROS2 connection |
| `web_app/js/virtual_joystick.js` | Touch-based joystick (fallback control) |
| `web_app/js/lidar_visualizer.js` | LiDAR point cloud AR overlay renderer |
| `web_app/js/app.js` | Main app orchestrator |
| `config/params.yaml` | Tunable parameters (speed, deadzone, etc.) |
| `launch/ar_controller.launch.py` | Main launch file (rosbridge + nodes) |

---

## 10. Android App 📱

**Native Android App** အနေနဲ့ သုံးပါ။ Web app က Android app ထဲမှာ bundled ဖြစ်နေလို့ HTTP server မလိုပါ။

### Android App Features (Browser ထက် ပိုကောင်းတဲ့အချက်များ)

| Feature | Browser | Android App |
|---------|---------|-------------|
| Camera Permission | HTTPS လို | Auto-request |
| Fullscreen | Manual | Auto-immersive |
| Screen Awake | No guarantee | Always on |
| Haptic Feedback | Limited | Native vibration |
| E-Stop Button | In-page only | Native FAB button |
| Connection Settings | URL ပြောင်းရ | Settings screen |
| Network Test | Manual | One-tap test |

### 10.1 Android Studio နဲ့ Build ခြင်း

```bash
# 1. Android Studio ကို ဖွင့်ပါ
# 2. "Open an existing project" → android_app/ folder ကို ရွေးပါ
# 3. Gradle sync ပြီးအောင် စောင့်ပါ

# Build APK:
# Menu → Build → Build Bundle(s) / APK(s) → Build APK(s)
# APK location: android_app/app/build/outputs/apk/debug/app-debug.apk
```

### 10.2 Command Line နဲ့ Build ခြင်း

```bash
cd ~/Desktop/rom_ar_app/android_app

# Debug APK build
./gradlew assembleDebug

# APK ကို ဖုန်းထဲ install
adb install app/build/outputs/apk/debug/app-debug.apk
```

### 10.3 App ကို အသုံးပြုခြင်း

1. **ဖုန်းမှာ App ဖွင့်ပါ** → "AR Robot Controller" icon
2. **Camera Permission** → Allow ပေးပါ
3. **Robot IP ထည့်ပါ** → Settings icon (⚙️) နှိပ်ပြီး Robot IP ထည့်ပါ
4. **Test Connection** → Robot PC reachable ဖြစ်/ မဖြစ် စစ်ပါ
5. **Save & Connect** → AR controller ကို auto-load လုပ်ပါမယ်

### 10.4 Android App Project Structure

```
android_app/
├── build.gradle.kts                         # Root build config
├── settings.gradle.kts                      # Project settings
├── gradle/wrapper/gradle-wrapper.properties # Gradle version
└── app/
    ├── build.gradle.kts                     # App build config
    ├── proguard-rules.pro                   # Release obfuscation rules
    └── src/main/
        ├── AndroidManifest.xml              # Permissions & activities
        ├── assets/
        │   └── web_app/                     # Bundled web app (no server needed)
        │       ├── index.html
        │       └── js/
        ├── java/com/arrobot/controller/
        │   ├── MainActivity.kt              # WebView + fullscreen + camera
        │   ├── ConnectionActivity.kt        # Robot IP/port settings
        │   └── WebAppInterface.kt           # JS ↔ Android bridge
        └── res/
            ├── layout/
            │   ├── activity_main.xml        # Main UI (WebView + overlays)
            │   └── activity_connection.xml  # Settings form
            ├── values/
            │   ├── strings.xml
            │   ├── colors.xml
            │   └── themes.xml
            └── drawable/                    # Icons (settings, refresh, etc.)
```

### 10.5 JavaScript ↔ Android Bridge

Web app ထဲက `AndroidBridge` ကို ခေါ်သုံးလို့ရပါတယ်:

```javascript
// Android app ဖြစ်/မဖြစ် စစ်ခြင်း
if (typeof AndroidBridge !== 'undefined' && AndroidBridge.isAndroidApp()) {
    // Native haptic feedback
    AndroidBridge.vibrate(50);
    
    // Toast message
    AndroidBridge.showToast("Connected to robot!");
    
    // Get saved ROS bridge URL
    const rosUrl = AndroidBridge.getROSBridgeURL();
    
    // Vibration pattern (delay, buzz, delay, buzz...)
    AndroidBridge.vibratePattern("0,50,100,50");
}
```
