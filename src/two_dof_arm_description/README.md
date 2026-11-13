# ESP32 to ROS2 — 5-DOF Right Arm IMU Motion Mimicking System

Real-time human right-arm motion mimicking using dual IMUs, ESP32, and ROS2 RViz visualization.

## System Architecture

```
ESP32 + 2× MPU-6500  →  Serial (115200 baud)  →  serial_imu_bridge
                                                        ↓
                                               /imu_shoulder/data
                                               /imu_elbow/data
                                                        ↓
                                              imu_to_joint_angles_v2
                                                        ↓
                                                  /joint_states
                                                        ↓
                                             robot_state_publisher → RViz2
```

## How It Works

1. **ESP32 Firmware:** Reads raw accel/gyro from 2× MPU-6500 via I2C (0x68, 0x69 on same bus). Runs Madgwick filter on-board → outputs quaternions at ~200Hz.
2. **Serial Bridge (`serial_imu_bridge`):** Parses quaternion data over serial, normalizes, publishes as ROS2 `sensor_msgs/Imu` messages.
3. **Kinematics (`imu_to_joint_angles_v2`):** Converts quaternions to 5 joint angles using relative quaternion computation for elbow isolation. Includes adaptive EMA smoothing.
4. **Visualization (`display.launch.py`):** Robot state publisher + RViz2 renders the URDF arm model driven by joint states.
5. **Debug (`imu_tf_broadcaster`):** Optional node that broadcasts raw IMU orientations as TF frames for visual axis verification.

## Hardware Setup

* **ESP32 Microcontroller**
* **IMU 1 (MPU-6500):** I2C address 0x68 (AD0 → GND) — strapped to upper arm
* **IMU 2 (MPU-6500):** I2C address 0x69 (AD0 → VCC) — strapped to forearm
* **I2C:** Both on same bus — SDA=21, SCL=22 (default Wire)

## Software Setup & Execution

### 1. Flash the ESP32
1. Open `esp32_imu_publisher.ino` in Arduino IDE
2. Install the `FastIMU` and `MadgwickAHRS` libraries
3. Connect ESP32 and select correct COM port
4. Upload. **Keep sensors STILL for first 6 seconds** (gyro calibration + Madgwick convergence)
5. You should see `READY` in Serial Monitor when calibration completes

### 2. Build the ROS2 Workspace
```bash
cd ~/Internship_ws
colcon build --packages-select two_dof_arm_description
source install/setup.bash
```

### 3. Run the System

**Option A: All-in-one script**
```bash
cd ~/Internship_ws
./run_all.sh /dev/ttyUSB0
```

**Option B: Separate terminals** (better for debugging)

**Terminal 1 — Visualizer:**
```bash
source install/setup.bash
ros2 launch two_dof_arm_description display.launch.py
```

**Terminal 2 — Serial Bridge:**
```bash
source install/setup.bash
ros2 run two_dof_arm_description serial_imu_bridge --ros-args -p port:=/dev/ttyUSB0
```

**Terminal 3 — Joint Angle Calculator:**
```bash
source install/setup.bash
ros2 run two_dof_arm_description imu_to_joint_angles_v2
```

**Terminal 4 (optional) — TF Debug:**
```bash
source install/setup.bash
ros2 run two_dof_arm_description imu_tf_broadcaster
```

### 4. Calibration
- Hold arm **straight down** during startup (~8 seconds)
- System auto-calibrates and prints `CALIBRATED`
- To recalibrate at any time:
  ```bash
  ros2 topic pub /calibrate_imu std_msgs/msg/Empty '{}' --once
  ```

### 5. Monitor Health
```bash
ros2 topic echo /imu_bridge/status        # Bridge diagnostics
ros2 topic echo /calibration_status        # Calibration state
ros2 topic echo /wrist_position            # FK wrist position
```
