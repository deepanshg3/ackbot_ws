# 🚗 Autonomous Navigation of an Ackermann Steering Based Electric Vehicle
### Using ROS 2 and LiDAR-Based Perception

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS_2-Humble_Hawksbill-blue?logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python&logoColor=white)
![Arduino](https://img.shields.io/badge/Arduino-Mega_2560-teal?logo=arduino&logoColor=white)
![Platform](https://img.shields.io/badge/Platform-NVIDIA_Jetson_Orin_Nano-green?logo=nvidia&logoColor=white)
![License](https://img.shields.io/badge/License-Academic-red)
![Status](https://img.shields.io/badge/Status-Completed_29_April_2026-brightgreen)

**B.Tech Final Year Project — Zakir Husain College of Engineering & Technology, AMU**

*Deepansh Gupta (22AEB555) · Sadat Ul Hasan (22AEB476)*

*Supervisor: Prof. Saleem Anwar Khan*

</div>

---

## 📌 Project Overview

This repository contains the complete ROS 2 software stack for a **hardware-validated autonomous navigation system** deployed on a physical Ackermann steering based electric vehicle. The project benchmarks two fundamentally different local control strategies — **MPPI (Model Predictive Path Integral)** vs **RPP (Regulated Pure Pursuit)** — on real hardware across two environments and two manoeuvre types, using a rigorous **2×2×2 factorial experimental design (N=40 trials)**.

> **This is not a simulation project.** Every result was obtained on physical hardware.

---

## 🏆 Key Results

| Metric | MPPI | RPP | Winner |
|--------|------|-----|--------|
| Mean CTE — Map 1 Forward | **0.027 m** | 0.044 m | ✅ MPPI |
| Mean CTE — Map 2 Forward | **0.026 m** | 0.050 m | ✅ MPPI |
| Final Position Error — Map 1 Forward | **0.143 m** | 0.198 m | ✅ MPPI |
| Final Yaw Error — Map 2 Backward | **0.068 rad** ⭐ | 0.185 rad | ✅ MPPI |
| Consistency (Std Dev) | **Low** | High | ✅ MPPI |

> ⭐ Best result in the entire 40-trial dataset: **MPPI backward parking in Map 2 — 0.068 rad yaw error**

**Key Finding:** Backward parking manoeuvres achieve lower final position error than forward parking for both controllers — a direct consequence of Ackermann reverse kinematics naturally aligning with the goal approach geometry.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        SENSORS                                  │
│  Ouster OS1-32 LiDAR ──(Ethernet)──┐                           │
│  IMU (internal to Ouster) ─────────┤                           │
│  Wheel Encoders ──────(USB)────────┤                           │
└────────────────────────────────────┼───────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                  JETSON ORIN NANO (ROS 2)                       │
│                                                                 │
│  /scan ──► SLAM Toolbox ──► /map                               │
│  /ouster/imu ──► EKF ──► /odom (filtered)                     │
│  /odom_raw ──────────────►                                     │
│  /odom ──► AMCL ──► /amcl_pose                                │
│  /map + /amcl_pose ──► Nav2 Stack                             │
│    ├── SmacPlannerHybrid (Reeds-Shepp)                        │
│    ├── MPPI / RPP Controller                                   │
│    └── Custom Behavior Tree (no Spin recovery)                │
│  /cmd_vel ──► AckermannController ──(USB Serial)──►           │
└────────────────────────────────────────────────────────────────┘
                                     ▼
┌─────────────────────────────────────────────────────────────────┐
│                    ARDUINO MEGA 2560                            │
│  PID Velocity Controller (50 Hz)                               │
│  Interrupt-driven Encoder Counting                             │
│  Servo PWM + Motor PWM Output                                  │
│  Encoder Delta Uplink (20 Hz)                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 Hardware

| Component | Specification |
|-----------|---------------|
| Chassis | Hiwonder Ackermann Steering (Aluminium) |
| LiDAR | Ouster OS1-32 (32-channel, 360°, Ethernet) |
| Edge Computer | NVIDIA Jetson Orin Nano (8GB) |
| Microcontroller | Arduino Mega 2560 |
| Motor Driver | Cytron MDD10A (Dual 10A) |
| Drive Motors | 12V DC Gear Motors with Hall Effect Encoders |
| Steering | MG996R Servo (11 kg·cm) |
| Battery | Hiwonder LiPo 11.1V 2200mAh |

**Physical Constraints:**
- Wheelbase: `0.213 m`
- Max steering angle: `±25°` (hardware cap due to asymmetric rack)
- Min turning radius: `~0.5 m`
- Max linear velocity: `0.20 m/s`

---

## 📦 Repository Structure

```
map_ws/src/
├── ackermann_control/                  # Primary ROS 2 Package
│   ├── ackermann_control/
│   │   ├── ackermann_controller_node.py   # cmd_vel → serial bridge
│   │   ├── odometry_node.py               # encoder ticks → /odom_raw
│   │   └── __init__.py
│   ├── config/
│   │   ├── ekf.yaml                       # Extended Kalman Filter config
│   │   ├── slam_toolbox.yaml              # SLAM Toolbox parameters
│   │   ├── nav2_params.yaml               # Nav2 MPPI parameters
│   │   ├── minimal_bt.xml                 # Custom Behavior Tree (no Spin)
│   │   └── rviz_slam.rviz                 # RViz layout
│   ├── launch/
│   │   ├── robot_core.launch.py           # EKF + Ouster + serial nodes
│   │   ├── mapping.launch.py              # SLAM Toolbox + robot_core
│   │   └── navigation.launch.py           # AMCL + Nav2 + robot_core
│   ├── maps/
│   │   ├── map.pgm                        # Saved occupancy grid image
│   │   └── map.yaml                       # Map metadata
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── ouster-ros/                         # Ouster LiDAR ROS 2 Driver
│   └── ...
└── arduino_code.txt                    # Arduino Mega firmware
```

---

## ⚙️ Software Stack

| Component | Package / Version |
|-----------|-------------------|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| SLAM | SLAM Toolbox |
| State Estimation | robot_localization (EKF) |
| Localization | AMCL |
| Global Planner | SmacPlannerHybrid (Reeds-Shepp) |
| Local Controller A | MPPI (Ackermann motion model) |
| Local Controller B | Regulated Pure Pursuit (RPP) |
| LiDAR Driver | ouster-ros (ROS 2) |

---

## 🚀 Getting Started

### Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# robot_localization
sudo apt install ros-humble-robot-localization

# Ouster ROS driver (clone into src/)
git clone https://github.com/ouster-lidar/ouster-ros.git
```

### Build

```bash
mkdir -p ~/map_ws/src
cd ~/map_ws/src

# Clone this repo
git clone https://github.com/YOUR_USERNAME/ackermann-autonomous-nav.git .

cd ~/map_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 🗺️ Usage

### Step 1 — Bring up the Robot Core
```bash
# Terminal 1: Start Ouster LiDAR driver + EKF + serial bridge
ros2 launch ackermann_control robot_core.launch.py
```

### Step 2 — Build a Map (SLAM Mapping Mode)
```bash
# Terminal 2: Launch SLAM Toolbox for mapping
ros2 launch ackermann_control mapping.launch.py

# Terminal 3: Teleoperate to build the map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/map_ws/src/ackermann_control/maps/map
```

### Step 3 — Autonomous Navigation
```bash
# Terminal 2: Launch full Nav2 stack with saved map
ros2 launch ackermann_control navigation.launch.py

# In RViz:
# 1. Set initial pose with "2D Pose Estimate"
# 2. Send goal with "Nav2 Goal" button
```

### Switch Between MPPI and RPP
Edit `config/nav2_params.yaml`:
```yaml
# For MPPI (default):
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Ackermann"

# For RPP:
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
  use_rotate_to_heading: false
  allow_reversing: true
```

---

## 📡 Serial Communication Protocol

### Command Packet — Jetson → Arduino (7 bytes)
```
[0xAA][0x55][0x03][speed: int8][servo_low: uint8][servo_high: uint8][XOR checksum]
```

### Odometry Packet — Arduino → Jetson (14 bytes)
```
[0x55][0xAA][0x0A][dL: int32 LE][dR: int32 LE][servo: uint16 LE][XOR checksum]
```

- Baud rate: `115200`
- Command rate: `30 Hz`
- Odometry uplink: `20 Hz`
- Safety timeout: `500 ms` (motors stop if no command received)

---

## 🤖 Arduino PID Parameters

```cpp
float Kp = 45.0;
float Ki = 12.0;
float Kd = 0.0;
float maxVel = 0.20;      // m/s
int   pwmCap = 160;       // out of 255 (safety limit)
unsigned long timeout = 500; // ms watchdog
```

---

## 📊 Experimental Design

```
                    40 Total Trials
                         │
            ┌────────────┴────────────┐
         Map 1 (Square)          Map 2 (Room)
              │                       │
       ┌──────┴──────┐         ┌──────┴──────┐
    Forward      Backward   Forward      Backward
    Parking      Parking    Parking      Parking
       │              │        │              │
  MPPI(5) RPP(5)  MPPI(5) RPP(5)  MPPI(5) RPP(5)  MPPI(5) RPP(5)
```

**Metrics:** Mean Cross-Track Error (CTE) · Final Position Error · Final Yaw Error

**Obstacle Config:**
- Map 1: 3 cardboard box obstacles (not part of saved map — detected live by costmap)
- Map 2: 1 cardboard box obstacle (same live detection approach)

---

## 📐 Key Kinematic Equations

**Steering angle from /cmd_vel:**
```
δ = arctan(ω · L / v)     [v must be signed for correct reverse handling]
```

**Odometry integration:**
```
x += d_center · cos(θ)
y += d_center · sin(θ)
θ += (v / L) · tan(δ) · dt
```

**Meters per encoder tick:**
```
d_per_tick = (2π × 0.0325) / 330 ≈ 0.000619 m/tick
```

---

## ⚠️ Known Issues & Workarounds

| Issue | Root Cause | Fix Applied |
|-------|-----------|-------------|
| Differential-spin during nav | Floating DIR pin on H-bridge from vibration | Soldered connections replacing jumper wires |
| Missing `/plan` in rosbag (RPP) | DDS IPC drop — RPP rarely re-publishes path | Re-recorded 15 affected trials under identical conditions |
| Vehicle trapped at goal | `stateful: true` in SimpleGoalChecker | Set `stateful: false` for MPPI config |
| IMU fighting encoder velocity | EKF fusing conflicting linear acceleration | Disabled `imu0` linear acceleration in EKF config |
| Nav2 Spin recovery on Ackermann | Default BT assumes differential drive | Custom `minimal_bt.xml` — BackUp only, no Spin |

---

## 📋 EKF Fusion Configuration Summary

```yaml
# Wheel odometry — trust linear x-velocity and yaw position
odom0_config: [false, false, false,
               false, false, true,    # yaw position
               true,  false, false,   # linear x velocity
               false, false, false,
               false, false, false]

# IMU — trust yaw velocity ONLY (linear acceleration OFF)
imu0_config: [false, false, false,
              false, false, false,
              false, false, false,
              false, false, false,
              false, false, true]     # yaw velocity only
```

---

## 🔭 Future Work

- [ ] 3D LiDAR SLAM (LIO-SAM / KISS-ICP) replacing 2D SLAM Toolbox
- [ ] Fully wireless architecture (Wi-Fi LiDAR + ESP32 motor bridge)
- [ ] MPC and learning-based controller comparison on same platform
- [ ] Custom PCB replacing breadboard + jumper wire prototype
- [ ] RTK-GPS integration for large-scale outdoor navigation
- [ ] Outdoor dynamic environment testing

---

## 📚 References

1. Macenski et al., *The ROS 2 Navigation System*, IEEE R&A Magazine, 2020
2. Macenski & Jambrecic, *SLAM Toolbox*, Journal of Field Robotics, 2021
3. Williams et al., *Information Theoretic MPC (MPPI)*, ICRA 2017
4. Macenski et al., *Survey of ROS 2 Mobile Robotics Algorithms*, R&AS 2023
5. Forte et al., *Context-Aware Navigation for AGVs*, IEEE RA-L 2025
6. Buchanan et al., *Racing With ROS 2*, arXiv:2311.14276, 2023

---

## 👥 Authors

| Name | Roll No. | Contribution |
|------|----------|--------------|
| Deepansh Gupta | 22AEB555 | Hardware integration, Arduino firmware, Nav2 tuning, experimental data collection |
| Sadat Ul Hasan | 22AEB476 | ROS 2 software architecture, SLAM implementation, controller benchmarking, data analysis |

**Supervisor:** Prof. Saleem Anwar Khan
**Department:** Mechanical Engineering, ZHCET, AMU
**Completed:** 29 April 2026

---

## 📄 License

This project is submitted as a B.Tech Final Year Project at Zakir Husain College of Engineering & Technology, Aligarh Muslim University. The code is made available for academic reference.

---

<div align="center">

*If this project helped you, please consider giving it a ⭐*

**Department of Mechanical Engineering · ZHCET · AMU · 2025–26**

</div>
