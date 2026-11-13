#!/usr/bin/env python3
"""
imu_to_joint_angles_v2.py
═══════════════════════════════════════════════════════════════════
ROS2 Node: 2 IMU Sensors → 5 Joint Angles + FK Wrist Position

PHASE 1 — IMU-ONLY FOUNDATION
──────────────────────────────
This is the core kinematics node for the right-arm motion mimicking
system. It converts raw IMU quaternions into URDF joint angles.

KEY DESIGN DECISIONS:
  1. RELATIVE QUATERNION FOR ELBOW
     The elbow IMU measures absolute orientation (includes shoulder).
     To get the TRUE elbow bend, we compute:
         q_elbow_relative = q_shoulder_inverse × q_elbow
     This gives us ONLY the rotation between upper arm and forearm.

  2. ZYX EULER DECOMPOSITION
     The URDF joint chain is Z (abduction) → Y (flexion) → X (rotation).
     quaternion_to_euler() extracts intrinsic XYZ = extrinsic ZYX,
     which directly maps to the URDF joint order.

  3. ADAPTIVE SMOOTHING
     Uses velocity-dependent EMA alpha:
     - Stationary: low alpha (heavy smoothing → no jitter)
     - Moving: high alpha (light smoothing → responsive)

  4. RIGHT ARM CONVENTION
     This models a RIGHT arm. Positive shoulder_flexion = forward raise.
     Positive elbow_flexion = bending arm toward shoulder.

JOINT → IMU MAPPING TABLE
──────────────────────────
Joint Name               | Source            | Euler Angle | URDF Axis
─────────────────────────┼───────────────────┼─────────────┼──────────
shoulder_abduction_joint | IMU 1 (absolute)  | yaw         | Z
shoulder_flexion_joint   | IMU 1 (absolute)  | pitch       | Y
shoulder_rotation_joint  | IMU 1 (absolute)  | roll        | X
elbow_flexion_joint      | IMU2 rel. to IMU1 | pitch       | Y
forearm_rotation_joint   | IMU2 rel. to IMU1 | yaw         | Z
═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
import math
import time

from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String
import numpy as np


# ─────────────────────────────────────────────────────────────────
# ARM GEOMETRY — measured from user's actual right arm
# ─────────────────────────────────────────────────────────────────
L_UPPER_ARM = 0.28    # shoulder to elbow (28 cm)
L_FOREARM   = 0.26    # elbow to wrist (26 cm)


# ─────────────────────────────────────────────────────────────────
# JOINT LIMITS — synced with URDF (human_arm_5dof.urdf)
# ─────────────────────────────────────────────────────────────────
LIMITS = {
    'shoulder_abduction_joint':  (-3.1416,  3.1416),   # Z-axis: full rotation
    'shoulder_flexion_joint':    (-1.5708,  2.6180),   # Y-axis: -90° to +150°
    'shoulder_rotation_joint':   (-1.5708,  1.5708),   # X-axis: ±90°
    'elbow_flexion_joint':       ( 0.0000,  2.5307),   # Y-axis: 0° to +145°
    'forearm_rotation_joint':    (-1.5708,  1.5708),   # Z-axis: ±90°
}

# Number of calibration samples (averaged for stable reference)
CALIBRATION_SAMPLES = 100

# ─────────────────────────────────────────────────────────────────
# ADAPTIVE SMOOTHING CONFIG
# ─────────────────────────────────────────────────────────────────
# Dead zone: Ignore changes smaller than ~1.5 degrees
DEAD_ZONE_RAD = math.radians(1.5)

# Adaptive EMA: alpha varies based on angular velocity
EMA_ALPHA_MIN = 0.15    # Heavy smoothing when stationary → kills jitter
EMA_ALPHA_MAX = 0.60    # Light smoothing when moving fast → responsive
VELOCITY_THRESHOLD = 2.0  # rad/s — alpha ramps from MIN to MAX over this range


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def quaternion_to_euler(x, y, z, w):
    """
    Quaternion (x,y,z,w) → Euler (roll, pitch, yaw) in radians.

    Convention: Intrinsic X→Y→Z rotation = Extrinsic Z→Y→X
    This matches the URDF joint chain:
        shoulder_abduction (Z) → shoulder_flexion (Y) → shoulder_rotation (X)

    Returns: (roll, pitch, yaw) where:
        roll  = rotation about X (maps to shoulder_rotation / forearm)
        pitch = rotation about Y (maps to shoulder_flexion / elbow_flexion)
        yaw   = rotation about Z (maps to shoulder_abduction / forearm_rotation)
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_multiply(q1, q2):
    """Multiply q1 ⊗ q2. Format: (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    )


def quaternion_conjugate(q):
    """Conjugate (inverse for unit quaternion): (x,y,z,w) → (-x,-y,-z,w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def quaternion_normalize(q):
    """Normalize quaternion to unit length."""
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-10:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)


def quaternion_average(quats):
    """
    Average a list of quaternions using simple mean + normalize.
    Good enough for small orientation spreads (calibration samples).
    """
    sx, sy, sz, sw = 0.0, 0.0, 0.0, 0.0
    ref = quats[0]
    for q in quats:
        dot = q[0]*ref[0] + q[1]*ref[1] + q[2]*ref[2] + q[3]*ref[3]
        if dot < 0:
            sx -= q[0]; sy -= q[1]; sz -= q[2]; sw -= q[3]
        else:
            sx += q[0]; sy += q[1]; sz += q[2]; sw += q[3]
    n = len(quats)
    return quaternion_normalize((sx/n, sy/n, sz/n, sw/n))


def compute_wrist_position(q1, q2, q3, q4, q5):
    """
    Forward Kinematics: 5 joint angles → wrist (x,y,z).
    Matches URDF chain: Z(q1) → Y(q2) → X(q3) → translate(-Z) → Y(q4) → Z(q5) → translate(-Z)

    Arm hangs straight down at rest (−Z direction).
    """
    def Rx(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])
    def Ry(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]])
    def Rz(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]])
    def Tz(d):
        return np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])

    # URDF chain: Rz(abd) @ Ry(flex) @ Rx(rot) @ Tz(-upper) @ Ry(elbow) @ Rz(forearm) @ Tz(-fore)
    T = (Rz(q1) @ Ry(q2) @ Rx(q3)
         @ Tz(-L_UPPER_ARM)
         @ Ry(q4) @ Rz(q5)
         @ Tz(-L_FOREARM))

    return T[0, 3], T[1, 3], T[2, 3]


class ImuToJointAnglesV2(Node):

    def __init__(self):
        super().__init__('imu_to_joint_angles_v2')

        self.get_logger().info(
            '\n'
            '╔════════════════════════════════════════════════════════╗\n'
            '║   5-DOF Right Arm IMU Node  (Phase 1 — IMU Only)      ║\n'
            '╠════════════════════════════════════════════════════════╣\n'
            '║  Shoulder: absolute orientation from IMU1              ║\n'
            '║  Elbow:    RELATIVE rotation (IMU2 vs IMU1)            ║\n'
            '║  → No shoulder leak into elbow joints!                 ║\n'
            '║  Direct quaternion input (no Euler conversion loss)    ║\n'
            '║  Adaptive EMA smoothing (velocity-dependent)           ║\n'
            '║  Multi-sample calibration ({} samples)                 ║\n'
            '╚════════════════════════════════════════════════════════╝'.format(
                CALIBRATION_SAMPLES
            )
        )

        # ── Calibration state ─────────────────────────────────────
        self.q_ref_shoulder = None
        self.q_ref_elbow    = None
        self.calibrated = False

        # Buffers for multi-sample calibration
        self.cal_shoulder_buf = []
        self.cal_elbow_buf = []

        # ── Latest IMU quaternions (x, y, z, w) ──────────────────
        self.q_shoulder = (0.0, 0.0, 0.0, 1.0)
        self.q_elbow    = (0.0, 0.0, 0.0, 1.0)

        # ── Current target angles (raw from IMUs) ─────────────────
        self.target_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

        # ── Smoothed output angles ────────────────────────────────
        self.smooth_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

        # ── Timing for adaptive smoothing ─────────────────────────
        self.last_publish_time = time.time()

        self.joint_names = [
            'shoulder_abduction_joint',
            'shoulder_flexion_joint',
            'shoulder_rotation_joint',
            'elbow_flexion_joint',
            'forearm_rotation_joint',
        ]

        # ── Calibration status ────────────────────────────────────
        self.cal_state = 'WARMING_UP'  # WARMING_UP → COLLECTING → CALIBRATED

        # ── ROS2 interfaces ──────────────────────────────────────
        self.create_subscription(Imu, '/imu_shoulder/data', self.shoulder_cb, 10)
        self.create_subscription(Imu, '/imu_elbow/data', self.elbow_cb, 10)
        self.create_subscription(Empty, '/calibrate_imu', self.calibrate_cb, 10)

        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_wrist  = self.create_publisher(PointStamped, '/wrist_position', 10)
        self.pub_cal_status = self.create_publisher(String, '/calibration_status', 10)

        self.fk_log_counter = 0
        self.timer = self.create_timer(0.01, self.publish_joint_states)  # 100 Hz
        self.cal_status_timer = self.create_timer(1.0, self.publish_cal_status)

        self.start_time = None
        self.warmup_time_sec = 7.0

        self.get_logger().info(
            f'⏳ Waiting {self.warmup_time_sec}s for IMU Madgwick filter to stabilize...'
        )
        self.get_logger().info(
            '──────────────────────────────────────────────────────────\n'
            '  RIGHT ARM — Direction Reference:\n'
            '  • Raise arm FORWARD  → shoulder_flexion (+pitch)\n'
            '  • Raise arm SIDEWAYS → shoulder_abduction (+yaw)\n'
            '  • Rotate upper arm   → shoulder_rotation (+roll)\n'
            '  • Bend elbow         → elbow_flexion (+pitch, 0→145°)\n'
            '  • Rotate forearm     → forearm_rotation (+yaw)\n'
            '──────────────────────────────────────────────────────────'
        )

    # ──────────────────────────────────────────────────────────────
    def calibrate(self):
        """Multi-sample calibration: average quaternions for stable ref."""
        if len(self.cal_shoulder_buf) >= CALIBRATION_SAMPLES and \
           len(self.cal_elbow_buf) >= CALIBRATION_SAMPLES:
            self.q_ref_shoulder = quaternion_average(self.cal_shoulder_buf)
            self.q_ref_elbow    = quaternion_average(self.cal_elbow_buf)
        else:
            # Fallback: single-sample calibration
            self.q_ref_shoulder = self.q_shoulder
            self.q_ref_elbow    = self.q_elbow

        self.calibrated = True
        self.cal_state = 'CALIBRATED'
        self.target_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.smooth_joints = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.get_logger().info(
            f'✅ CALIBRATED! (from {len(self.cal_shoulder_buf)} samples)\n'
            f'   Shoulder ref: ({self.q_ref_shoulder[0]:.4f}, '
            f'{self.q_ref_shoulder[1]:.4f}, '
            f'{self.q_ref_shoulder[2]:.4f}, '
            f'{self.q_ref_shoulder[3]:.4f})\n'
            f'   Elbow ref:    ({self.q_ref_elbow[0]:.4f}, '
            f'{self.q_ref_elbow[1]:.4f}, '
            f'{self.q_ref_elbow[2]:.4f}, '
            f'{self.q_ref_elbow[3]:.4f})\n'
            '── System is now LIVE — move your arm! ──────────────────'
        )

    def calibrate_cb(self, msg):
        """Manual recalibration via /calibrate_imu topic."""
        self.get_logger().info('🔄 Manual recalibration! Hold arm straight down...')
        self.calibrated = False
        self.cal_state = 'COLLECTING'
        self.cal_shoulder_buf.clear()
        self.cal_elbow_buf.clear()

    def publish_cal_status(self):
        """Publish calibration state for external monitoring."""
        msg = String()
        if self.cal_state == 'WARMING_UP':
            elapsed = 0.0
            if self.start_time is not None:
                elapsed = time.time() - self.start_time
            remaining = max(0, self.warmup_time_sec - elapsed)
            msg.data = f'WARMING_UP remaining={remaining:.1f}s'
        elif self.cal_state == 'COLLECTING':
            s_count = len(self.cal_shoulder_buf)
            e_count = len(self.cal_elbow_buf)
            msg.data = f'COLLECTING shoulder={s_count}/{CALIBRATION_SAMPLES} elbow={e_count}/{CALIBRATION_SAMPLES}'
        else:
            msg.data = 'CALIBRATED'
        self.pub_cal_status.publish(msg)

    def get_calibrated_quat(self, q_raw, q_ref):
        """
        Get the GLOBAL world rotation applied since calibration:
            q_calibrated = q_raw ⊗ q_ref_conjugate
        """
        return quaternion_multiply(q_raw, quaternion_conjugate(q_ref))

    # ──────────────────────────────────────────────────────────────
    def shoulder_cb(self, msg):
        q = msg.orientation
        self.q_shoulder = (q.x, q.y, q.z, q.w)

        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        if elapsed < self.warmup_time_sec:
            return

        if not self.calibrated:
            if len(self.cal_shoulder_buf) == 0:
                self.cal_state = 'COLLECTING'
                self.get_logger().info(
                    '⏳ Warmup complete! Hold arm STRAIGHT DOWN for calibration... '
                    f'(collecting {CALIBRATION_SAMPLES} samples)'
                )

            self.cal_shoulder_buf.append(self.q_shoulder)
            if (len(self.cal_shoulder_buf) >= CALIBRATION_SAMPLES and
                    len(self.cal_elbow_buf) >= CALIBRATION_SAMPLES):
                self.calibrate()
            elif len(self.cal_shoulder_buf) % 25 == 0:
                self.get_logger().info(
                    f'   Calibrating... {len(self.cal_shoulder_buf)}/'
                    f'{CALIBRATION_SAMPLES} shoulder samples'
                )
            return

        # ── SHOULDER: absolute orientation relative to calibration ─
        q_cal = self.get_calibrated_quat(self.q_shoulder, self.q_ref_shoulder)
        roll, pitch, yaw = quaternion_to_euler(*q_cal)

        lo, hi = LIMITS['shoulder_abduction_joint']
        self.target_joints[0] = clamp(yaw, lo, hi)

        lo, hi = LIMITS['shoulder_flexion_joint']
        self.target_joints[1] = clamp(pitch, lo, hi)

        lo, hi = LIMITS['shoulder_rotation_joint']
        self.target_joints[2] = clamp(roll, lo, hi)

    # ──────────────────────────────────────────────────────────────
    def elbow_cb(self, msg):
        q = msg.orientation
        self.q_elbow = (q.x, q.y, q.z, q.w)

        if self.start_time is None:
            return  # Wait for shoulder to set the global start_time

        elapsed = time.time() - self.start_time
        if elapsed < self.warmup_time_sec:
            return

        if not self.calibrated:
            self.cal_elbow_buf.append(self.q_elbow)
            return

        # ── ELBOW: RELATIVE rotation between forearm and upper arm ─
        q_shoulder_cal = self.get_calibrated_quat(
            self.q_shoulder, self.q_ref_shoulder)

        q_elbow_cal = self.get_calibrated_quat(
            self.q_elbow, self.q_ref_elbow)

        # q_relative = q_shoulder_inverse × q_elbow
        # This gives us ONLY the bend at the elbow, not the shoulder!
        q_relative = quaternion_multiply(
            quaternion_conjugate(q_shoulder_cal), q_elbow_cal)

        roll, pitch, yaw = quaternion_to_euler(*q_relative)

        lo, hi = LIMITS['elbow_flexion_joint']
        self.target_joints[3] = clamp(pitch, lo, hi)

        lo, hi = LIMITS['forearm_rotation_joint']
        self.target_joints[4] = clamp(yaw, lo, hi)

    # ──────────────────────────────────────────────────────────────
    def publish_joint_states(self):
        """Apply dead zone, adaptive EMA smoothing, and publish."""

        now = time.time()
        dt = now - self.last_publish_time
        self.last_publish_time = now

        for i in range(5):
            diff = self.target_joints[i] - self.smooth_joints[i]

            # Dead-zone: if movement is smaller than threshold, ignore it
            if abs(diff) > DEAD_ZONE_RAD:
                # Adaptive alpha: more smoothing when still, less when moving
                velocity = abs(diff) / max(dt, 0.001)
                alpha = EMA_ALPHA_MIN + (EMA_ALPHA_MAX - EMA_ALPHA_MIN) * min(
                    velocity / VELOCITY_THRESHOLD, 1.0)
                self.smooth_joints[i] += alpha * diff

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(self.smooth_joints)
        msg.velocity = [0.0] * 5
        msg.effort   = [0.0] * 5
        self.pub_joints.publish(msg)

        # FK wrist position
        wx, wy, wz = compute_wrist_position(*self.smooth_joints)
        wrist_msg = PointStamped()
        wrist_msg.header.stamp = msg.header.stamp
        wrist_msg.header.frame_id = 'base_link'
        wrist_msg.point.x = wx
        wrist_msg.point.y = wy
        wrist_msg.point.z = wz
        self.pub_wrist.publish(wrist_msg)

        # Log every ~2s
        self.fk_log_counter += 1
        if self.fk_log_counter >= 200:
            self.fk_log_counter = 0
            self.get_logger().info(
                f'Wrist: ({wx:.3f}, {wy:.3f}, {wz:.3f})  |  '
                f'abd={math.degrees(self.smooth_joints[0]):.1f}° '
                f'flex={math.degrees(self.smooth_joints[1]):.1f}° '
                f'rot={math.degrees(self.smooth_joints[2]):.1f}° '
                f'elb={math.degrees(self.smooth_joints[3]):.1f}° '
                f'fore={math.degrees(self.smooth_joints[4]):.1f}°'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuToJointAnglesV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
