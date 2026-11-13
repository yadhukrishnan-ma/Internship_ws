#!/bin/bash
# ═══════════════════════════════════════════════════════════════
# run_all.sh — Launch the complete IMU arm mimicking system
# Phase 1+: IMU Foundation + ArUco Ground Truth
#
# Usage: ./run_all.sh [port] [--no-aruco]
#   port:       Serial port for ESP32 (default: /dev/ttyUSB0)
#   --no-aruco: Skip ArUco pose publisher (run IMU-only)
#
# This runs 5 nodes:
#   1. RViz + robot_state_publisher (display.launch.py)
#   2. Serial IMU bridge (ESP32 → ROS2)
#   3. Joint angle calculator (IMU → URDF joints)
#   4. TF broadcaster (raw IMU debug visualization)
#   5. ArUco pose publisher (webcam → ground truth poses)
# ═══════════════════════════════════════════════════════════════

set -e

PORT=${1:-/dev/ttyUSB0}
USE_ARUCO=true

# Parse args
for arg in "$@"; do
    case "$arg" in
        --no-aruco) USE_ARUCO=false ;;
    esac
done

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "════════════════════════════════════════════════════════════"
echo "  5-DOF Right Arm IMU Mimic System + ArUco Ground Truth"
echo "  Serial port: $PORT"
echo "  ArUco: $( [ "$USE_ARUCO" = true ] && echo 'ENABLED ✅' || echo 'DISABLED' )"
echo "════════════════════════════════════════════════════════════"

# Track PIDs for clean shutdown
PIDS=()

cleanup() {
    echo ""
    echo "Shutting down all nodes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null
    echo "All nodes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

NODE_COUNT=4
[ "$USE_ARUCO" = true ] && NODE_COUNT=5

# ── 1. Launch RViz + robot_state_publisher ────────────────────
echo "[1/${NODE_COUNT}] Starting RViz + Robot State Publisher..."
ros2 launch two_dof_arm_description display.launch.py use_gui:=false &
PIDS+=($!)
sleep 4

# ── 2. Start Serial IMU Bridge ───────────────────────────────
echo "[2/${NODE_COUNT}] Starting Serial IMU Bridge on $PORT..."
ros2 run two_dof_arm_description serial_imu_bridge \
    --ros-args -p port:="$PORT" -p baudrate:=115200 &
PIDS+=($!)
sleep 1

# ── 3. Start Joint Angle Calculator ──────────────────────────
echo "[3/${NODE_COUNT}] Starting Joint Angle Calculator..."
ros2 run two_dof_arm_description imu_to_joint_angles_v2 &
PIDS+=($!)

# ── 4. Start TF Broadcaster (debug) ──────────────────────────
echo "[4/${NODE_COUNT}] Starting IMU TF Broadcaster (debug)..."
ros2 run two_dof_arm_description imu_tf_broadcaster &
PIDS+=($!)

# ── 5. Start ArUco Pose Publisher (ground truth) ─────────────
if [ "$USE_ARUCO" = true ]; then
    sleep 1
    echo "[5/${NODE_COUNT}] Starting ArUco Pose Publisher (webcam)..."
    ros2 run aruco_pose_publisher aruco_node \
        --ros-args \
        -p marker_size:=0.037 \
        -p camera_index:=0 \
        -p show_preview:=true \
        -p camera_matrix_path:="/home/dell/Aruco marker/camera_matrix.npy" \
        -p dist_coeffs_path:="/home/dell/Aruco marker/dist_coeffs.npy" &
    PIDS+=($!)
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo "  ✅ All ${NODE_COUNT} nodes running!"
echo "  • Hold arm STRAIGHT DOWN during the first ~8 seconds"
echo "  • System auto-calibrates then goes LIVE"
if [ "$USE_ARUCO" = true ]; then
echo "  • ArUco preview window shows marker detection"
echo "  • Check /aruco/accuracy topic for FK vs ArUco comparison"
fi
echo "  • Press Ctrl+C to stop all nodes"
echo "  • To recalibrate: ros2 topic pub /calibrate_imu std_msgs/msg/Empty '{}' --once"
echo "════════════════════════════════════════════════════════════"

wait "${PIDS[@]}"
