#!/usr/bin/env python3
"""
serial_imu_bridge.py
─────────────────────────────────────────────────────────────────
ROS2 Node: Serial Port (ESP32) → sensor_msgs/msg/Imu
Author: Generated for ROS2 Robotics Project

Reads the raw comma-separated quaternion strings coming from the
ESP32 via UART/Serial, and re-publishes them as ROS2 Imu messages
on /imu_shoulder/data and /imu_elbow/data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import threading
import sys

class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.pub_shoulder = self.create_publisher(Imu, '/imu_shoulder/data', 10)
        self.pub_elbow = self.create_publisher(Imu, '/imu_elbow/data', 10)

        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f"Successfully connected to ESP32 on {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            sys.exit(1)

        # Run reading in a separate thread so it doesn't block ROS spinning
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.start()

    def serial_read_loop(self):
        while self.running and rclpy.ok():
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if not line:
                        continue
                    
                    # Expected format: q1_w,q1_x,q1_y,q1_z,q2_w,q2_x,q2_y,q2_z
                    parts = line.split(',')
                    if len(parts) == 8:
                        q1_w, q1_x, q1_y, q1_z, q2_w, q2_x, q2_y, q2_z = map(float, parts)
                        
                        now = self.get_clock().now().to_msg()

                        # Shoulder IMU Message
                        msg_shoulder = Imu()
                        msg_shoulder.header.stamp = now
                        msg_shoulder.header.frame_id = 'imu_shoulder_link'
                        msg_shoulder.orientation.w = q1_w
                        msg_shoulder.orientation.x = q1_x
                        msg_shoulder.orientation.y = q1_y
                        msg_shoulder.orientation.z = q1_z
                        self.pub_shoulder.publish(msg_shoulder)

                        # Elbow IMU Message
                        msg_elbow = Imu()
                        msg_elbow.header.stamp = now
                        msg_elbow.header.frame_id = 'imu_elbow_link'
                        msg_elbow.orientation.w = q2_w
                        msg_elbow.orientation.x = q2_x
                        msg_elbow.orientation.y = q2_y
                        msg_elbow.orientation.z = q2_z
                        self.pub_elbow.publish(msg_elbow)
                        
            except ValueError:
                # Catch float parsing errors or malformed lines
                # self.get_logger().debug("Malformed serial data received.")
                pass
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.running = False
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def __del__(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        if hasattr(self, 'read_thread') and self.read_thread.is_alive():
            self.read_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = SerialImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Serial IMU Bridge.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
