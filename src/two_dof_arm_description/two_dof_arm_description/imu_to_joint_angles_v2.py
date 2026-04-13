#!/usr/bin/env python3
"""
imu_to_joint_angles_v2.py
═══════════════════════════════════════════════════════════════════
UPGRADED ROS2 Node: 2 IMU Sensors → 5 Joint Angles
Author : Upgraded for human-like 5-DOF arm

WHAT THIS NODE DOES
───────────────────
                IMU 1 (Shoulder)                IMU 2 (Elbow)
                /imu_shoulder/data              /imu_elbow/data
                        │                               │
                        ▼                               ▼
              Extract 3 Euler angles        Compute RELATIVE rotation
              from quaternion               between both IMU quaternions
                        │                               │
               ┌────────┴───────┐              ┌────────┴───────┐
               ▼        ▼       ▼              ▼                ▼
          abduction  flexion rotation    elbow_flexion  forearm_rotation
               │        │       │              │                │
               └────────┴───────┴──────────────┴────────────────┘
                                        │
                                        ▼
                         /joint_states  (5 angles published)
                                        │
                                        ▼
                              robot_state_publisher
                                        │
                                        ▼
                                 RViz / Gazebo

JOINT → IMU MAPPING TABLE
──────────────────────────
Joint Name               | Source  | Euler Angle  | Axis
─────────────────────────┼─────────┼──────────────┼──────
shoulder_abduction_joint | IMU 1   | roll         | X
shoulder_flexion_joint   | IMU 1   | pitch        | Y
shoulder_rotation_joint  | IMU 1   | yaw          | Z
elbow_flexion_joint      | Rel.    | pitch_rel     | Y
forearm_rotation_joint   | Rel.    | roll_rel      | Z

MATH: HOW WE GET 5 ANGLES FROM 2 QUATERNIONS
─────────────────────────────────────────────

STEP 1 — Shoulder angles (from IMU1 quaternion directly):
  An IMU gives absolute orientation as quaternion q = (x, y, z, w).
  We convert to Euler angles (roll, pitch, yaw) using ZYX convention:

    roll  = atan2( 2(wy + xz),  1 - 2(y² + z²) )   → abduction
    pitch = asin ( 2(wx - yz) )                      → flexion
    yaw   = atan2( 2(wz + xy),  1 - 2(x² + z²) )    → rotation

  These match shoulder DOF axes: X(abduction), Y(flexion), Z(rotation)

STEP 2 — Relative rotation for elbow (from both quaternions):
  The elbow angles describe how the FOREARM moves RELATIVE TO the
  upper arm — not its absolute world orientation.

  q_relative = q_shoulder_conjugate ⊗ q_elbow

  The conjugate of q_shoulder is (−x, −y, −z, +w).
  Multiplying gives the rotation "seen from the upper arm's frame".

  From q_relative:
    pitch_rel → elbow_flexion     (how much elbow is bent)
    roll_rel  → forearm_rotation  (pronation / supination)

USAGE
─────
  Terminal 1:  ros2 launch two_dof_arm_description display_5dof.launch.py
  Terminal 2:  ros2 run two_dof_arm_description imu_to_joint_angles_v2
  Terminal 3:  ros2 topic echo /joint_states   (verify 5 joints publishing)
═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Imu, JointState


# ─────────────────────────────────────────────────────────────────
# JOINT LIMITS — must exactly match the URDF values
# ─────────────────────────────────────────────────────────────────
LIMITS = {
    'shoulder_abduction_joint':  (-0.5236,  3.1416),   # -30°  to +180°
    'shoulder_flexion_joint':    (-1.0472,  3.1416),   # -60°  to +180°
    'shoulder_rotation_joint':   (-1.2217,  1.5708),   # -70°  to +90°
    'elbow_flexion_joint':       ( 0.0000,  2.5307),   #   0°  to +145°
    'forearm_rotation_joint':    (-1.5708,  1.5708),   # -90°  to +90°
}


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp value between lo and hi."""
    return max(lo, min(hi, value))


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    """
    Convert quaternion (x, y, z, w) → Euler angles (roll, pitch, yaw).

    Uses ZYX convention (most common for IMU sensors):
      1. Yaw   around world Z-axis first
      2. Pitch around new Y-axis
      3. Roll  around final X-axis

    Returns angles in RADIANS.

    BEGINNER NOTE:
      A quaternion is a 4-number encoding of rotation that avoids
      "gimbal lock" (a problem where two rotation axes line up and
      you lose a degree of freedom). We convert it to the familiar
      roll/pitch/yaw (RPY) representation for joint angle mapping.
    """
    # Roll (rotation about X-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotation about Y-axis)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))   # safe clamp for asin
    pitch = math.asin(sinp)

    # Yaw (rotation about Z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions: q_result = q1 ⊗ q2

    Quaternion multiplication composes two rotations.
    Result = apply q2 first, then q1.

    q1 = (x1, y1, z1, w1)
    q2 = (x2, y2, z2, w2)

    Formula (Hamilton product):
      w = w1*w2 - x1*x2 - y1*y2 - z1*z2
      x = w1*x2 + x1*w2 + y1*z2 - z1*y2
      y = w1*y2 - x1*z2 + y1*w2 + z1*x2
      z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,   # x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,   # y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,   # z
        w1*w2 - x1*x2 - y1*y2 - z1*z2    # w
    )


def quaternion_conjugate(q):
    """
    Compute the conjugate (inverse) of a unit quaternion.

    For a unit quaternion (|q| = 1):
      conjugate(x, y, z, w) = (-x, -y, -z, +w)

    Multiplying q ⊗ conjugate(q) = identity (no rotation).
    This is used to "undo" the shoulder rotation from the forearm
    quaternion, giving us the RELATIVE rotation of the forearm
    with respect to the upper arm.
    """
    x, y, z, w = q
    return (-x, -y, -z, w)


class ImuToJointAnglesV2(Node):
    """
    Upgraded ROS2 node: 2 IMUs → 5 joint angles for human-like arm.

    Subscribes:
        /imu_shoulder/data   (sensor_msgs/Imu)
        /imu_elbow/data      (sensor_msgs/Imu)

    Publishes:
        /joint_states        (sensor_msgs/JointState)
        → 5 joints: shoulder_abduction, shoulder_flexion,
                    shoulder_rotation, elbow_flexion, forearm_rotation
    """

    def __init__(self):
        super().__init__('imu_to_joint_angles_v2')

        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════╗\n'
            '║   5-DOF Human Arm IMU Node  (v2)         ║\n'
            '╠══════════════════════════════════════════╣\n'
            '║  Subscribing to:                         ║\n'
            '║    /imu_shoulder/data                    ║\n'
            '║    /imu_elbow/data                       ║\n'
            '║  Publishing to:                          ║\n'
            '║    /joint_states  (5 joints)             ║\n'
            '╚══════════════════════════════════════════╝'
        )

        # ── Store latest IMU quaternions ───────────────────────────
        # Format: (x, y, z, w) — matches ROS sensor_msgs/Imu.orientation
        self.q_shoulder = (0.0, 0.0, 0.0, 1.0)   # identity = no rotation
        self.q_elbow    = (0.0, 0.0, 0.0, 1.0)

        # ── Current joint angles (radians) ────────────────────────
        self.shoulder_abduction = 0.0
        self.shoulder_flexion   = 0.0
        self.shoulder_rotation  = 0.0
        self.elbow_flexion      = 0.0
        self.forearm_rotation   = 0.0

        # ── Subscribers ────────────────────────────────────────────
        self.sub_shoulder = self.create_subscription(
            Imu, '/imu_shoulder/data',
            self.shoulder_callback, 10)

        self.sub_elbow = self.create_subscription(
            Imu, '/imu_elbow/data',
            self.elbow_callback, 10)

        # ── Publisher ──────────────────────────────────────────────
        self.pub_joints = self.create_publisher(JointState, '/joint_states', 10)

        # ── Publish timer: 50 Hz ───────────────────────────────────
        self.timer = self.create_timer(0.02, self.publish_joint_states)

    # ──────────────────────────────────────────────────────────────
    def shoulder_callback(self, msg: Imu):
        """
        Process shoulder IMU data → 3 shoulder joint angles.

        The shoulder IMU is mounted on the upper arm.
        Its ABSOLUTE orientation in the world frame directly encodes
        all 3 shoulder angles via Euler angle decomposition.

        Mapping (ZYX Euler decomposition):
          roll  (about X) → shoulder_abduction_joint
          pitch (about Y) → shoulder_flexion_joint
          yaw   (about Z) → shoulder_rotation_joint
        """
        q = msg.orientation
        self.q_shoulder = (q.x, q.y, q.z, q.w)

        # Decompose quaternion into 3 Euler angles
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        # Map to joint angles with clamping to URDF limits
        lo, hi = LIMITS['shoulder_abduction_joint']
        self.shoulder_abduction = clamp(roll, lo, hi)

        lo, hi = LIMITS['shoulder_flexion_joint']
        self.shoulder_flexion = clamp(pitch, lo, hi)

        lo, hi = LIMITS['shoulder_rotation_joint']
        self.shoulder_rotation = clamp(yaw, lo, hi)

    # ──────────────────────────────────────────────────────────────
    def elbow_callback(self, msg: Imu):
        """
        Process elbow IMU data → 2 elbow/forearm joint angles.

        By user request: We treat the IMUs as INDEPENDENT. 
        We do NOT subtract the shoulder's orientation. 
        Roll/Pitch of the elbow IMU maps directly to the elbow joints.
        """
        q = msg.orientation
        self.q_elbow = (q.x, q.y, q.z, q.w)

        # Extract Euler angles directly from the elbow IMU quaternion
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)

        # Map to joint angles with clamping
        lo, hi = LIMITS['elbow_flexion_joint']
        self.elbow_flexion = clamp(pitch, lo, hi)

        # Map roll to forearm rotation
        lo, hi = LIMITS['forearm_rotation_joint']
        self.forearm_rotation = clamp(roll, lo, hi)

    # ──────────────────────────────────────────────────────────────
    def publish_joint_states(self):
        """
        Publish all 5 joint angles as a JointState message at 50 Hz.

        The robot_state_publisher reads this and updates the TF tree,
        which causes the 3D robot model to move in RViz and Gazebo.

        CRITICAL: Joint names here MUST exactly match the URDF joint names.
        Even one typo will cause that joint to silently stay at 0°.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''

        msg.name = [
            'shoulder_abduction_joint',
            'shoulder_flexion_joint',
            'shoulder_rotation_joint',
            'elbow_flexion_joint',
            'forearm_rotation_joint',
        ]

        msg.position = [
            self.shoulder_abduction,
            self.shoulder_flexion,
            self.shoulder_rotation,
            self.elbow_flexion,
            self.forearm_rotation,
        ]

        # Not controlling velocity or effort, just position
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort   = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.pub_joints.publish(msg)


# ─────────────────────────────────────────────────────────────────
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
