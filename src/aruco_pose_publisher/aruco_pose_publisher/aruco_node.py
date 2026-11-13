#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Int32, Float32
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class ArucoPosePublisher(Node):

    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # ── ROS2 Publishers ──────────────────────────────────────
        self.pose_pub     = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        self.id_pub       = self.create_publisher(Int32,       '/aruco_marker_id', 10)
        self.distance_pub = self.create_publisher(Float32,     '/aruco_distance', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Load calibration ─────────────────────────────────────
        try:
            self.camera_matrix = np.load('/home/dell/Aruco marker/camera_matrix.npy')
            self.dist_coeffs   = np.load('/home/dell/Aruco marker/dist_coeffs.npy')
            self.get_logger().info('✅ Calibration loaded!')
        except FileNotFoundError:
            self.get_logger().error('❌ camera_matrix.npy not found!')
            raise

        # ── ArUco setup ──────────────────────────────────────────
        ARUCO_DICT  = cv2.aruco.DICT_5X5_50
        self.MARKER_SIZE = 0.037          # ← your verified marker size in meters

        aruco_dict  = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        parameters  = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

        half = self.MARKER_SIZE / 2.0
        self.marker_3d_corners = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0],
        ], dtype=np.float32)

        # ── Camera ───────────────────────────────────────────────
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('❌ Cannot open webcam!')
            raise RuntimeError('Webcam not found')

        # ── Timer: runs detection at ~30Hz ───────────────────────
        self.timer = self.create_timer(0.033, self.detect_and_publish)
        self.get_logger().info('🎥 ArUco Pose Publisher started!')
        self.get_logger().info('   Listening for markers ID 5 and ID 9...')

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for corner, marker_id in zip(corners, ids):
                success, rvec, tvec = cv2.solvePnP(
                    self.marker_3d_corners,
                    corner[0],
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                if not success:
                    continue

                x = tvec[0][0]
                y = tvec[1][0]
                z = tvec[2][0]

                # ── Convert rvec to quaternion ──────────────────
                rot_matrix, _ = cv2.Rodrigues(rvec)
                r = Rotation.from_matrix(rot_matrix)
                quat = r.as_quat()   # [qx, qy, qz, qw]

                now = self.get_clock().now().to_msg()

                # ── Publish PoseStamped ─────────────────────────
                pose_msg = PoseStamped()
                pose_msg.header.stamp    = now
                pose_msg.header.frame_id = 'camera_frame'
                pose_msg.pose.position.x = x
                pose_msg.pose.position.y = y
                pose_msg.pose.position.z = z
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]
                self.pose_pub.publish(pose_msg)

                # ── Publish marker ID ───────────────────────────
                id_msg = Int32()
                id_msg.data = int(marker_id[0])
                self.id_pub.publish(id_msg)

                # ── Publish distance (Z) ────────────────────────
                dist_msg = Float32()
                dist_msg.data = float(z)
                self.distance_pub.publish(dist_msg)

                # ── Publish TF transform ────────────────────────
                tf_msg = TransformStamped()
                tf_msg.header.stamp    = now
                tf_msg.header.frame_id = 'camera_frame'
                tf_msg.child_frame_id  = f'aruco_marker_{marker_id[0]}'
                tf_msg.transform.translation.x = x
                tf_msg.transform.translation.y = y
                tf_msg.transform.translation.z = z
                tf_msg.transform.rotation.x = quat[0]
                tf_msg.transform.rotation.y = quat[1]
                tf_msg.transform.rotation.z = quat[2]
                tf_msg.transform.rotation.w = quat[3]
                self.tf_broadcaster.sendTransform(tf_msg)

                # ── Draw on screen ──────────────────────────────
                cv2.drawFrameAxes(frame, self.camera_matrix,
                                  self.dist_coeffs, rvec, tvec,
                                  self.MARKER_SIZE * 0.5)

                mid_x = int(np.mean(corner[0][:, 0]))
                mid_y = int(np.mean(corner[0][:, 1]))
                cv2.putText(frame, f"ID:{marker_id[0]}",
                            (mid_x-30, mid_y-50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
                cv2.putText(frame, f"Z:{z:.3f}m ({z*100:.1f}cm)",
                            (mid_x-60, mid_y-25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                cv2.putText(frame, f"X:{x:+.3f} Y:{y:+.3f}",
                            (mid_x-60, mid_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255,255,0), 1)

                self.get_logger().info(
                    f'ID:{marker_id[0]}  '
                    f'X:{x:+.4f}  Y:{y:+.4f}  Z:{z:+.4f}m'
                )
        else:
            cv2.putText(frame, "No marker detected",
                        (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0,0,255), 2)

        cv2.imshow("ArUco ROS2 Publisher", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()