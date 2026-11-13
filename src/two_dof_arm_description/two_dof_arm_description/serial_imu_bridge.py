#!/usr/bin/env python3
"""
serial_imu_bridge.py
═════════════════════════════════════════════════════════════════
ROS2 Node: Serial Port (ESP32) → sensor_msgs/msg/Imu

Reads quaternion data directly from ESP32 via serial.

SUPPORTED FORMATS:
  Legacy  (8 fields): q1w,q1x,q1y,q1z,q2w,q2x,q2y,q2z
  New     (10 fields): seq,timestamp_us,q1w,q1x,q1y,q1z,q2w,q2x,q2y,q2z

Features:
  • Automatic format detection (backward compatible)
  • Dropped packet detection via sequence numbers
  • Latency estimation via ESP32 timestamps
  • Health diagnostics published to /imu_bridge/status
  • Quaternion normalization safety check
═════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import serial
import threading
import sys
import time
import math


class SerialImuBridge(Node):
    def __init__(self):
        super().__init__('serial_imu_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        self.pub_shoulder = self.create_publisher(Imu, '/imu_shoulder/data', 10)
        self.pub_elbow = self.create_publisher(Imu, '/imu_elbow/data', 10)
        self.pub_status = self.create_publisher(String, '/imu_bridge/status', 10)

        try:
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to ESP32 on {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            sys.exit(1)

        # Flush any boot garbage from ESP32
        time.sleep(0.5)
        self.serial_port.reset_input_buffer()

        # Wait for READY signal from ESP32
        self.get_logger().info("Waiting for ESP32 READY signal...")
        self._wait_for_ready()

        # ── Diagnostics counters ─────────────────────────────────
        self.msg_count = 0
        self.skip_count = 0
        self.drop_count = 0
        self.parse_errors = 0
        self.last_seq = -1
        self.format_detected = None  # 'legacy' or 'new'
        self.diag_start_time = time.time()

        # Run reading in a daemon thread so it doesn't block ROS spinning
        self.running = True
        self.read_thread = threading.Thread(target=self.serial_read_loop, daemon=True)
        self.read_thread.start()

        # Periodic diagnostics (every 5 seconds)
        self.diag_timer = self.create_timer(5.0, self.publish_diagnostics)

    def _wait_for_ready(self):
        """Wait up to 15 seconds for the ESP32 READY signal."""
        deadline = time.time() + 15.0
        while time.time() < deadline:
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                self.get_logger().info(f"  ESP32: {line}")
                if line == 'READY':
                    self.get_logger().info("✅ ESP32 is READY — starting data acquisition.")
                    return
            except Exception:
                pass

        self.get_logger().warn(
            "⚠ No READY signal received (timeout 15s). "
            "Proceeding anyway — ESP32 may use old firmware."
        )

    def serial_read_loop(self):
        while self.running and rclpy.ok():
            try:
                raw = self.serial_port.readline()
                if not raw:
                    continue

                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                parts = line.split(',')

                # ── Auto-detect and parse format ─────────────────
                if len(parts) == 10:
                    # NEW format: seq, timestamp_us, q1w..q1z, q2w..q2z
                    if self.format_detected is None:
                        self.format_detected = 'new'
                        self.get_logger().info(
                            "📡 Detected NEW format (seq + timestamp + 8 quaternion values)"
                        )

                    seq = int(parts[0])
                    # timestamp_us = int(parts[1])  # available for latency analysis
                    q1w, q1x, q1y, q1z = float(parts[2]), float(parts[3]), float(parts[4]), float(parts[5])
                    q2w, q2x, q2y, q2z = float(parts[6]), float(parts[7]), float(parts[8]), float(parts[9])

                    # Dropped packet detection
                    if self.last_seq >= 0:
                        gap = seq - self.last_seq - 1
                        if gap > 0:
                            self.drop_count += gap
                            if self.drop_count <= 10:
                                self.get_logger().warn(
                                    f"Dropped {gap} packet(s) (seq {self.last_seq} → {seq})"
                                )
                    self.last_seq = seq

                elif len(parts) == 8:
                    # LEGACY format: q1w,q1x,q1y,q1z,q2w,q2x,q2y,q2z
                    if self.format_detected is None:
                        self.format_detected = 'legacy'
                        self.get_logger().info(
                            "📡 Detected LEGACY format (8 quaternion values, no seq/timestamp)"
                        )
                    q1w, q1x, q1y, q1z, q2w, q2x, q2y, q2z = map(float, parts)

                else:
                    self.skip_count += 1
                    if self.skip_count <= 5:
                        self.get_logger().warn(f"Skipping non-data line: '{line[:60]}'")
                    continue

                # ── Normalize quaternions (safety check) ─────────
                norm1 = math.sqrt(q1w*q1w + q1x*q1x + q1y*q1y + q1z*q1z)
                norm2 = math.sqrt(q2w*q2w + q2x*q2x + q2y*q2y + q2z*q2z)

                if norm1 < 0.5 or norm2 < 0.5:
                    continue  # Skip bad data

                q1w /= norm1; q1x /= norm1; q1y /= norm1; q1z /= norm1
                q2w /= norm2; q2x /= norm2; q2y /= norm2; q2z /= norm2

                now = self.get_clock().now().to_msg()

                # Shoulder IMU Message
                msg_shoulder = Imu()
                msg_shoulder.header.stamp = now
                msg_shoulder.header.frame_id = 'imu_shoulder_link'
                msg_shoulder.orientation.w = q1w
                msg_shoulder.orientation.x = q1x
                msg_shoulder.orientation.y = q1y
                msg_shoulder.orientation.z = q1z
                self.pub_shoulder.publish(msg_shoulder)

                # Elbow IMU Message
                msg_elbow = Imu()
                msg_elbow.header.stamp = now
                msg_elbow.header.frame_id = 'imu_elbow_link'
                msg_elbow.orientation.w = q2w
                msg_elbow.orientation.x = q2x
                msg_elbow.orientation.y = q2y
                msg_elbow.orientation.z = q2z
                self.pub_elbow.publish(msg_elbow)

                # Log first few messages + periodic status
                self.msg_count += 1
                if self.msg_count <= 3 or self.msg_count % 500 == 0:
                    r1, p1, y1 = quat_to_euler_deg(q1x, q1y, q1z, q1w)
                    r2, p2, y2 = quat_to_euler_deg(q2x, q2y, q2z, q2w)
                    self.get_logger().info(
                        f"[msg #{self.msg_count}] "
                        f"shoulder: R={r1:.1f}° P={p1:.1f}° Y={y1:.1f}°  |  "
                        f"elbow: R={r2:.1f}° P={p2:.1f}° Y={y2:.1f}°"
                    )

            except ValueError:
                self.parse_errors += 1
                self.get_logger().debug("Float parse error on serial line.")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                self.running = False
                break
            except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")

    def publish_diagnostics(self):
        """Publish health status every 5 seconds."""
        elapsed = time.time() - self.diag_start_time
        rate = self.msg_count / elapsed if elapsed > 0 else 0.0

        status = (
            f"rate={rate:.0f}Hz msgs={self.msg_count} "
            f"drops={self.drop_count} skips={self.skip_count} "
            f"parse_err={self.parse_errors} fmt={self.format_detected or 'unknown'}"
        )

        msg = String()
        msg.data = status
        self.pub_status.publish(msg)

        # Log periodically
        if self.msg_count > 0 and self.msg_count % 1000 < 500:
            self.get_logger().info(f"📊 Bridge health: {status}")

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def quat_to_euler_deg(x, y, z, w):
    """Quaternion → Euler degrees (for logging only, NOT in data path)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy))

    return roll, pitch, yaw


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
