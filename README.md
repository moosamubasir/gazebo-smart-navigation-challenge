> **Intelligent Grid Navigation with Vision & avoiding protocol**
> 

---

> 📸 **[PICTURE — Arena setup here]**
> 

---

## 📌 Overview

An autonomous robot navigation system where a differential-drive robot navigates a **5×5 grid arena**, collects bonus points, avoids obstacles, and reaches the goal — implemented in both **Gazebo simulation** and on a **real physical robot**.

An overhead laptop webcam handles computer vision (HSV colour detection & obstacle recognition), A* pathfinding plans the optimal route, and commands are sent wirelessly to a Raspberry Pi-powered wheeled robot for real-time execution.

---

## 🏆 Scoring System

> Bonus sheets are placed randomly in the arena each round.
> 

| Sheet Color | Points | Priority |
| --- | --- | --- |
| 🔴 Red Sheet | **10 pts** | High |
| 🔵 Blue Sheet | **5 pts** | Medium |
| ⬛ Obstacle (Cardboard Box) | Penalty / Block | Avoid |

---

## 🎯 How it works:

- An overhead laptop webcam captures a live top-down view of the 5×5 grid arena
- OpenCV detects and classifies grid cells using HSV colour detection — red bonus sheets (10 pts), blue bonus sheets (5 pts), and cardboard box obstacles
- A* pathfinding computes the optimal route — collecting all bonuses first, then navigating to the goal
- The laptop-side vision system sends movement commands over WebSocket to the Raspberry Pi on the robot
- The Raspberry Pi translates commands and forwards motor instructions to the Arduino via serial (115200 baud) as 2-byte packets `[CMD, SPEED]`
- The Arduino drives two DC gear motors through the motor driver
- A live web dashboard (Flask + WebSocket) streams the warped grid view, score, phase, obstacle count, and event log in real time

---

## 🏗️ System Architecture

```
Laptop (Vision + Planning)
│
├── camera_vision.py       → Overhead webcam, HSV colour detection
├── grid_manager.py        → 5×5 grid state, obstacle/bonus tracking
├── autonomous_bridge.py   → A* pathfinding, decision logic
├── web_dashboard.py       → Flask + WebSocket live dashboard
└── hardware_bridge.py     → Serial communication → RPi/Arduino
         │
         │ Serial (USB) / WebSocket (Wi-Fi)
         ▼
Raspberry Pi (On-Robot)
         │
         ▼
Arduino (Motor Control & I/O)
```

---

## 🧠 Core Features

| Feature | Details |
| --- | --- |
| 🗺️ Path Planning | A* algorithm with bonus-collection-first routing |
| 👁️ Computer Vision | Overhead webcam, HSV-based color detection (OpenCV) |
| 📦 Obstacle Detection | Cardboard box obstacles detected via vision |
| 🎯 Bonus Collection | Red (10 pts) & Blue (5 pts) sheets prioritized in routing |
| 🔌 Serial Communication | RPi ↔ Laptop via `/dev/ttyUSB0` at baud rate `115200` |
| 🌐 Web Dashboard | Flask + WebSocket live grid monitoring |
| 🧩 Modular Design | ROS2 Humble package structure, easy to extend |

---

## 🗂️ Project Structure

```
gazebo-smart-navigation-challenge/
├── arduino code/
│   └── arduino.ino                  # Motor control firmware
├── real robot/                       # ROS2 package: path_ws
│   ├── autonomous_bridge.py         # RPi ↔ laptop WebSocket bridge + A* logic
│   ├── camera_vision.py             # Webcam feed & HSV colour detection
│   ├── grid_manager.py              # 5×5 grid state management
│   ├── hardware_bridge.py           # Serial/WebSocket communication
│   ├── stream.py                    # Camera stream handler
│   ├── web_dashboard.py             # Live monitoring dashboard
│   ├── gazebo_tutorials.launch.py
│   ├── package.xml
│   └── setup.py
├── simulation/                       # ROS2 package: gazebo_tutorial
│   ├── __init__.py
│   ├── bonus_grid.world             # Gazebo world file (5×5 arena)
│   ├── bonus_randomizer.py          # Randomises bonus positions each run
│   ├── collector_controller.py      # Simulation robot controller
│   ├── gazebo_tutorial              # ROS2 package entry
│   ├── gazebo_tutorials.launch.py
│   ├── lidar.xacro                  # Robot URDF with RPLidar
│   └── setup.py
└── README.md
```

---

## 🔧 Hardware Stack

| Component | Role |
| --- | --- |
| Raspberry Pi 4 Model B | Main compute unit on robot |
| Arduino Uno  | Motor control & I/O |
| Laptop Webcam (overhead) | Computer vision for grid detection |
| DC Gear Motors × 2 | Drive wheels |
| raspberry type-c adapter | Power supply |
| 2-wheels | to move the robot |

### 🔌 Serial Connection

| Parameter | Value |
| --- | --- |
| Port | `/dev/ttyUSB0` |
| Baud Rate | `115200` |

---

## 💻 Software Stack

| Software | Version | Purpose |
| --- | --- | --- |
| Ubuntu 22.04 (WSL2) | — | Development OS |
| ROS2 Humble | Humble | Robot middleware |
| Gazebo Classic | — | Simulation environment |
| Python 3 | 3.x | Main programming language |
| OpenCV (`opencv-contrib-python`) | 4.x | Computer vision, colour detection |
| PySerial | Latest | Serial communication with Arduino |
| NumPy | Latest | Array operations |
| Flask + Flask-SocketIO | Latest | Live web dashboard |
| Arduino IDE | Latest | Motor controller firmware |

---

## 📡 Serial Command Protocol

Commands are sent as **2-byte binary packets**: `[CMD_BYTE, SPEED_BYTE]` at **115200 baud**.

| CMD Byte | Action | SPEED Byte |
| --- | --- | --- |
| :--------: | -------- | ------------ |
| `F` | Move Forward | PWM value (40–220) |
| `B` | Move Backward | PWM value (40–220) |
| `L` | Turn Left (burst) | PWM value (40–220) |
| `R` | Turn Right (burst) | PWM value (40–220) |
| `S` | Stop | `0` |

---

## 🏟️ Arena Setup

| Property | Value |
| --- | --- |
| Grid Size | 5×5 cells |
| Boundaries | Cardboard walls |
| Obstacles | Cardboard boxes (randomly placed) |
| Red Bonus Sheets | 10 points each |
| Blue Bonus Sheets | 5 points each |
| Camera Position | Overhead laptop webcam (bird's-eye view) |

> 📸 **[PICTURE — Arena layout photo]**
> 

---

## 🚀 Setup & Installation — Simulation

### Prerequisites

- Ubuntu 22.04 / 24.04 (or WSL2)
- ROS2 Humble installed
- Gazebo Classic installed

### Build & Run

```bash
# Navigate to your ROS2 workspace src folder
cd ~/ros2_ws/src

# navigate your workspace root and build
colcon build --packages-select gazebo_tutorial

# Source the workspace
source install/setup.bash

# Launch the simulation
ros2 launch gazebo_tutorial gazebo_tutorials.launch.py
```

> 📸 **[PICTURE / VIDEO — Gazebo simulation running]**
> 

---

## 🚀 Setup & Installation — Real Robot

### Prerequisites

- Raspberry Pi 4 with Raspberry Pi OS
- Arduino Uno connected via USB
- Overhead USB webcam connected to laptop
- Python 3 and pip installed
- ROS2 Humble installed

### Step 1 — Clone the Repository (on laptop)

```bash
git clone https://github.com/PukyBots/gazebo-smart-navigation-challenge.git
cd gazebo-smart-navigation-challenge
```

### Step 2 — Install Python Dependencies

```bash
pip3 install opencv-contrib-python numpy flask flask-socketio pyserial
```

### Step 3 — Build ROS2 Workspace

```bash
cd ~/path_ws
colcon build
source install/setup.bash
```

### Step 4 — Upload Arduino Code

1. Open `arduino code/arduino.ino` in Arduino IDE
2. Connect Arduino Uno via USB to laptop
3. Select **Tools → Board → Arduino Uno**
4. Select correct port under **Tools → Port**
5. Click **Upload** and wait for *Done uploading.*
6. Disconnect from laptop, connect to Raspberry Pi

### Step 5 — Find Arduino Port on Raspberry Pi

```bash
ls /dev/ttyUSB*
```

Give permission if needed:

```bash
sudo chmod 666 /dev/ttyUSB0
```

### Step 7 — Run the System

On the **laptop** (vision, A*, dashboard):

```bash
ros2 run follow_grid camera_vision
```

Open the live dashboard in your browser:

```
http://localhost:5000
```

On the **Raspberry Pi** (to launch/start the robot):

```bash
ros2 launch follow_grid gazebo_tutorials.launch.py
```

> 📸 **[PICTURE / VIDEO — Real robot running in arena]**
> 

---

## 🌐 Web Dashboard

The dashboard (`web_dashboard.py`) provides:

| Feature | Description |
| --- | --- |
| 📷 Live Feed | Overhead camera feed with warped grid overlay |
| 🗺️ Grid View | Robot position, obstacles, bonus sheets |
| 🛤️ Path Display | Current A* planned path visualisation |
| 📊 Score Tracker | Live bonus points collected |
| 📋 Event Log | Real-time system messages |
| 🔴🔵 Cell Detection | Red/Blue bonus cells and orange obstacle cells |

> 📸 **[PICTURE — Web dashboard screenshot]**
> 

---

## 🛠️ Troubleshooting

| Issue | Fix |
| --- | --- |
| `Permission denied /dev/ttyUSB0` | Run `sudo chmod 666 /dev/ttyUSB0` |
| Colours not detected correctly | Adjust HSV values in `camera_vision.py` |
| Grid misaligned with camera | Re-run perspective warp calibration |
| Robot not moving | Check baud rate is `115200`, verify USB cable |
| ROS2 nodes not found | Run `source install/setup.bash` first |
| Camera window doesn't open | Make sure webcam is plugged in before running |

---

## ✅ Project Status

### Done

- Gazebo simulation with 5×5 bonus grid and differential-drive robot
- Overhead camera HSV colour detection for grid classification
- A* pathfinding with bonus-first routing
- Real robot motor control via Arduino serial protocol
- WebSocket communication between laptop and Raspberry Pi
- Live web dashboard with real-time grid overlay and scoring

### 📋 To Do

- Fine-tune HSV thresholds for varying lighting conditions
- Improve motor tick calibration for precise grid movement
- Migrate to full ROS2 topic-based architecture for real robot

---

## 📋 Project Info

| Field | Details |
| --- | --- |
| **Team** | Mohammed Afeez& Moosa Mubasir |
| **Program** | TCE Internship — Sahyadri College of Engineering & Management |
| **Mentor** | Pulkit Garg, Technical Career Education |
| **Organization** | TCE(Technical Career Education) |
