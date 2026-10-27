"""Publish real Kobuki encoder odometry and TF on native Windows.

By default this receives telemetry only. --enable-motion also accepts /cmd_vel
for a bounded keyboard test with command and telemetry timeouts.
Wheel dimensions and encoder scale follow kobuki_core's DiffDrive defaults:
https://github.com/kobuki-base/kobuki_core/blob/devel/src/driver/diff_drive.cpp
"""
import argparse
import math
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from serial.tools import list_ports
from tf2_ros import TransformBroadcaster

from kobuki_monitor import PacketReader, decode_payload
from kobuki_motion import MotionController, STOP_FRAME, disable_console_selection


TICK_METERS = 0.035 * 0.002436916871363930187454
WHEEL_SEPARATION = 0.23


def encoder_delta(current, previous):
    return (current - previous + 32768) % 65536 - 32768


class EncoderOdometry:
    def __init__(self):
        self.previous = None
        self.x = self.y = self.yaw = 0.0
        self.linear = self.angular = 0.0
        self.rebases = 0

    def update(self, timestamp, left, right):
        current = (timestamp, left, right)
        if self.previous is None:
            self.previous = current
            return True
        old_time, old_left, old_right = self.previous
        elapsed_ms = (timestamp - old_time) % 65536
        if elapsed_ms == 0:
            return False
        dt = elapsed_ms / 1000.0
        dl = encoder_delta(left, old_left) * TICK_METERS
        dr = encoder_delta(right, old_right) * TICK_METERS
        self.previous = current
        # Reset/long interruptions or implausible counter jumps establish a
        # new baseline instead of producing a fictitious displacement.
        if dt > 0.5 or max(abs(dl), abs(dr)) / dt > 2.0:
            self.linear = self.angular = 0.0
            self.rebases += 1
            return False
        distance = (dl + dr) / 2.0
        angle = (dr - dl) / WHEEL_SEPARATION
        if abs(angle) < 1e-9:
            local_x, local_y = distance, 0.0
        else:
            local_x = distance * math.sin(angle) / angle
            local_y = distance * (1.0 - math.cos(angle)) / angle
        self.x += math.cos(self.yaw) * local_x - math.sin(self.yaw) * local_y
        self.y += math.sin(self.yaw) * local_x + math.cos(self.yaw) * local_y
        self.yaw = (self.yaw + angle + math.pi) % math.tau - math.pi
        self.linear, self.angular = distance / dt, angle / dt
        return True


def make_odometry(state, stamp):
    msg = Odometry()
    msg.header.stamp = stamp
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    msg.pose.pose.position.x, msg.pose.pose.position.y = state.x, state.y
    msg.pose.pose.orientation.z = math.sin(state.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(state.yaw / 2.0)
    msg.twist.twist.linear.x = state.linear
    msg.twist.twist.angular.z = state.angular
    # Initial uncertainty estimates; z, roll and pitch are unobserved.
    for index, value in zip((0, 7, 14, 21, 28, 35),
                            (0.01, 0.01, 1e6, 1e6, 1e6, 0.04)):
        msg.pose.covariance[index] = value
        msg.twist.covariance[index] = value
    transform = TransformStamped()
    transform.header = msg.header
    transform.child_frame_id = msg.child_frame_id
    transform.transform.translation.x = state.x
    transform.transform.translation.y = state.y
    transform.transform.rotation = msg.pose.pose.orientation
    return msg, transform


class KobukiOdometryNode(Node):
    def __init__(self, port, enable_motion=False):
        super().__init__("windows_kobuki_odometry")
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.broadcaster = TransformBroadcaster(self)
        self.parser = PacketReader()
        self.odometry = EncoderOdometry()
        self.samples = 0
        self.latest = {}
        self.motion = MotionController() if enable_motion else None
        self.last_report = time.monotonic()
        self.connection = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=0.1)
        self.connection.dtr = False
        self.connection.rts = False
        self.connection.port = port
        try:
            self.connection.open()
        except serial.SerialException:
            self.destroy_node()
            raise
        self.timer = self.create_timer(0.01, self.read_serial)
        mode = "keyboard motion enabled" if enable_motion else "receive only"
        self.get_logger().info(f"Reading Kobuki {port}; publishing /odom and TF odom -> base_link ({mode})")
        if enable_motion:
            self.command_subscription = self.create_subscription(Twist, "/cmd_vel", self.command_received, 1)
            self.drive_timer = self.create_timer(0.02, self.send_motion)
            self.get_logger().info("Motion ready: /cmd_vel; limits 0.12 m/s and 0.6 rad/s; command timeout 0.4 s. Waiting for keyboard input.")

    def command_received(self, msg):
        self.motion.accept(msg.linear.x, msg.angular.z, time.monotonic())

    def send_motion(self):
        frame = self.motion.frame(time.monotonic())
        if self.connection.write(frame) != len(frame):
            raise serial.SerialTimeoutException("Incomplete motor command write")

    def read_serial(self):
        available = self.connection.in_waiting
        if available:
            payloads = self.parser.feed(self.connection.read(min(available, 4096)))
            for payload in payloads:
                try:
                    data = decode_payload(payload)
                except ValueError:
                    continue
                if "robot_timestamp_ms" not in data:
                    continue
                self.latest = data
                if self.motion is not None:
                    self.motion.update_telemetry(data, time.monotonic())
                if not self.odometry.update(data["robot_timestamp_ms"],
                                            data["left_encoder"], data["right_encoder"]):
                    continue
                stamp = self.get_clock().now().to_msg()
                msg, transform = make_odometry(self.odometry, stamp)
                self.publisher.publish(msg)
                self.broadcaster.sendTransform(transform)
                self.samples += 1
                if self.samples == 1:
                    self.get_logger().info(f"First odometry: battery={data['battery_v']:.1f} V, left={data['left_encoder']}, right={data['right_encoder']}")
        now = time.monotonic()
        if now - self.last_report >= 5:
            if self.samples:
                state = self.odometry
                self.get_logger().info(f"Published {self.samples} odometry samples; x={state.x:.3f} m, y={state.y:.3f} m, yaw={math.degrees(state.yaw):.1f} deg; checksum errors={self.parser.bad_checksums}; rebases={state.rebases}")
                if self.motion is not None:
                    self.get_logger().info(f"Motion: {self.motion.reason}; target v={self.motion.linear:.2f} m/s, w={self.motion.angular:.2f} rad/s")
            else:
                self.get_logger().warning("No base telemetry yet. Check Kobuki power, its USB connection, and whether another program has its COM port open.")
            self.last_report = now

    def close(self):
        try:
            if self.motion is not None:
                self.motion.stop()
                if self.connection.is_open:
                    for _ in range(3):
                        try:
                            self.connection.write(STOP_FRAME)
                            time.sleep(0.01)
                        except serial.SerialException:
                            break
        finally:
            self.connection.close()
            self.destroy_node()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="Auto-selects the USB serial number containing KOBUKI")
    ap.add_argument("--enable-motion", action="store_true", help="Accept /cmd_vel with speed limits and timeout stops")
    args, ros_args = ap.parse_known_args()
    candidates = [p.device for p in list_ports.comports() if "KOBUKI" in p.hwid.upper()]
    port = args.port or (candidates[0] if len(candidates) == 1 else None)
    if not port:
        ap.error("No unique Kobuki adapter. Specify --port COMn.")
    rclpy.init(args=ros_args)
    node = None
    restore_console = disable_console_selection() if args.enable_motion else lambda: None
    code = 0
    try:
        node = KobukiOdometryNode(port, enable_motion=args.enable_motion)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except serial.SerialException as exc:
        print(f"Kobuki serial error: {exc}", flush=True)
        code = 1
    finally:
        try:
            if node is not None:
                node.close()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            restore_console()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
