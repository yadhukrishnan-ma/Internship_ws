#!/usr/bin/env python3
"""
imu_tf_broadcaster.py
═════════════════════════════════════════════════════════════════
ROS2 Node: Broadcast raw IMU orientations as TF frames

PURPOSE:
  Visualize IMU quaternions directly in RViz as TF coordinate axes.
  This lets you visually verify:
    • Is the IMU rotating in the correct direction?
    • Does the X/Y/Z axis of the IMU match what you expect?
    • Is there drift visible in the raw orientation?

PUBLISHED TF FRAMES:
  world → imu_shoulder_raw    (raw shoulder IMU orientation)
  world → imu_elbow_raw       (raw elbow IMU orientation)

These appear as RGB axis triads in RViz when TF display is enabled.
Compare them against the URDF arm links to verify alignment.
═════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class ImuTfBroadcaster(Node):

    def __init__(self):
        super().__init__('imu_tf_broadcaster')

        self.get_logger().info(
            '\n'
            '╔════════════════════════════════════════════════════════╗\n'
            '║   IMU TF Broadcaster — Debug Visualization            ║\n'
            '╠════════════════════════════════════════════════════════╣\n'
            '║  Publishing raw IMU orientations as TF frames:        ║\n'
            '║    world → imu_shoulder_raw                           ║\n'
            '║    world → imu_elbow_raw                              ║\n'
            '║  Enable TF display in RViz to see colored axes.       ║\n'
            '╚════════════════════════════════════════════════════════╝'
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, '/imu_shoulder/data', self.shoulder_cb, 10)
        self.create_subscription(Imu, '/imu_elbow/data', self.elbow_cb, 10)

        self.msg_count = 0

    def shoulder_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_shoulder_raw'

        # Place the debug frame offset from the arm so it doesn't overlap
        t.transform.translation.x = 0.3
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.4   # same height as shoulder

        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.tf_broadcaster.sendTransform(t)

        self.msg_count += 1
        if self.msg_count <= 2:
            self.get_logger().info(
                f'Broadcasting imu_shoulder_raw: '
                f'q=({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, '
                f'{msg.orientation.z:.3f}, {msg.orientation.w:.3f})'
            )

    def elbow_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'imu_elbow_raw'

        # Place below and to the side
        t.transform.translation.x = 0.3
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.1   # below shoulder debug frame

        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IMU TF Broadcaster stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
