# 🤖 Human Arm Simulation using ROS2 + IMU (5-DOF)

A **ROS2-based human-like robotic arm simulation** that mimics real arm motion using **IMU sensors**.

This project models a **5-DOF arm (Shoulder + Elbow)** and maps **real-world IMU orientation data → joint movements in RViz/Gazebo**.

---

## 🚀 Features

* ✅ 5-DOF Human Arm Model (URDF)
* ✅ Realistic Shoulder (3 DOF) + Elbow (2 DOF)
* ✅ IMU-based motion tracking
* ✅ ROS2 integration (Humble compatible)
* ✅ RViz visualization with live joint control
* ✅ Scalable to real hardware (ESP32 + IMU)

---

## 🧠 System Overview

```
        IMU (Shoulder)          IMU (Elbow)
             │                      │
             ▼                      ▼
      Quaternion Data        Quaternion Data
             │                      │
             └──────► ROS2 Node ◄───┘
                    (imu_to_joint_angles_v2)
                             │
                             ▼
                     /joint_states
                             │
                             ▼
               robot_state_publisher
                             │
                             ▼
                    RViz / Gazebo
```

---

## 🦾 Degrees of Freedom (DOF)

### Shoulder (3 DOF)

* Abduction / Adduction → X-axis
* Flexion / Extension → Y-axis
* Rotation → Z-axis

### Elbow (2 DOF)

* Flexion → Y-axis
* Forearm Rotation → Z-axis

---

## 📁 Project Structure

```
two_dof_arm_description/
├── urdf/
│   └── human_arm_5dof.urdf
├── launch/
│   └── display.launch.py
├── rviz/
│   └── arm_view.rviz
├── two_dof_arm_description/
│   └── imu_to_joint_angles_v2.py
├── config/
├── package.xml
├── setup.py
└── CMakeLists.txt (if applicable)
```

---

## ⚙️ Installation & Setup

### 1️⃣ Clone Repository

```bash
cd ~/Internship_ws/src
git clone <your-repo-link>
```

---

### 2️⃣ Build Workspace

```bash
cd ~/Internship_ws
colcon build
```

---

### 3️⃣ Source Workspace

```bash
source install/setup.bash
```

---

## ▶️ Run Simulation

### Launch RViz with Arm Model

```bash
ros2 launch two_dof_arm_description display.launch.py
```

---

### Run IMU Processing Node

```bash
ros2 run two_dof_arm_description imu_to_joint_angles_v2
```

---

### Check Joint States

```bash
ros2 topic echo /joint_states
```

---

## 🎮 Testing Without IMU (Important)

Use GUI sliders:

* Launch:

  ```bash
  ros2 launch two_dof_arm_description display.launch.py
  ```
* Move joints using **joint_state_publisher_gui**

You should see **5 sliders**:

* shoulder_abduction_joint
* shoulder_flexion_joint
* shoulder_rotation_joint
* elbow_flexion_joint
* forearm_rotation_joint

---

## 🔌 Hardware Integration (ESP32 + IMU)

### Required Components

* ESP32
* 2x IMU Sensors (e.g., MPU6050 / BNO055)

### Topics Used

| Sensor       | Topic                |
| ------------ | -------------------- |
| Shoulder IMU | `/imu_shoulder/data` |
| Elbow IMU    | `/imu_elbow/data`    |

---

## 🧮 Core Logic

### Shoulder Angles (Absolute)

From quaternion → Euler:

```
roll  → shoulder_abduction
pitch → shoulder_flexion
yaw   → shoulder_rotation
```

---

### Elbow Angles (Relative Rotation)

```
q_relative = q_shoulder⁻¹ ⊗ q_elbow
```

From relative quaternion:

```
pitch → elbow_flexion
roll  → forearm_rotation
```

---

## 📊 Joint Limits

| Joint              | Range        |
| ------------------ | ------------ |
| shoulder_abduction | -30° to 180° |
| shoulder_flexion   | -60° to 180° |
| shoulder_rotation  | -70° to 90°  |
| elbow_flexion      | 0° to 145°   |
| forearm_rotation   | -90° to 90°  |

---

## ⚠️ Common Issues & Fixes

### ❌ Only 2 joints showing

👉 Fix:

```bash
rm -rf build install log
colcon build
source install/setup.bash
```

---

### ❌ Package not found

👉 Fix:

```bash
source install/setup.bash
```

---

### ❌ No movement in RViz

* Check `/joint_states`
* Ensure IMU topics are publishing

---

## 📌 Future Improvements

* Add wrist (2 DOF → total 7 DOF)
* Integrate MoveIt2 for motion planning
* Add real-time control with PID
* Export to Gazebo with physics simulation

---

## 👨‍💻 Author

**Yadhu Krishnan**
Robotics Enthusiast | ROS2 | Embedded Systems

---

## 🌟 Project Goal

To build a **real-time human motion tracking robotic arm** using:

* ROS2
* IMU Sensors
* Embedded Systems (ESP32)

---

## ⭐ Support

If you found this project useful:

* ⭐ Star the repo
* 🍴 Fork it
* 📢 Share with others

---
