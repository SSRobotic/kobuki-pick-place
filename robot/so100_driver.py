"""Real SO-100 arm driver: Feetech STS3215 servos (IDs 1-6) <-> ROS 2.

Same topics as fake_arm.py, so arm_control.py and mission_nav2.py work unchanged:
  subscribes /arm/joint_command (sensor_msgs/JointState, joint angles of kobuki_x2.urdf / so_arm100.xml)
  publishes  /joint_states      (position in rad, effort = servo load -1..1)
Servo steps <-> joint angles come from so100_calibration.yaml, written by so100_calibrate.py (run it first).
Start robot_model.launch.py with arm:=none so nothing else publishes the arm joints.
    pixi run python robot\\so100_driver.py                    (Windows, port found automatically)
    python3 so100_driver.py --ros-args -p port:=/dev/ttyACM0  (Ubuntu)
    ... -p mock:=true     no arm: virtual servos, to try everything at home
Safety: moves at a limited speed, keeps the commanded angles inside the URDF limits, and leaves the torque ON when
you stop it (the arm would fall otherwise) - hold the arm and use so100_calibrate.py --relax to let it go limp.
"""
import math
import os

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState

from feetech import RAD_PER_STEP, STEPS, BusError, FeetechBus, VirtualServos

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
IDS = dict(zip(JOINTS, range(1, 7)))           # LeRobot SO-100 numbering
LIMITS = {"shoulder_pan": (-1.92, 1.92), "shoulder_lift": (-3.32, 0.174), "elbow_flex": (-0.174, 3.14),
          "wrist_flex": (-1.66, 1.66), "wrist_roll": (-2.79, 2.79), "gripper": (-0.174, 1.75)}
REST = {"shoulder_pan": 0.0, "shoulder_lift": -3.32, "elbow_flex": 3.11,
        "wrist_flex": 1.18, "wrist_roll": -1.5708, "gripper": -0.17}
HERE = os.path.dirname(os.path.abspath(__file__))
CAL_FILE = os.path.join(HERE, "so100_calibration.yaml")


def load_calibration(path=CAL_FILE):
    """{joint: (zero_step, sign)} - angle = sign * (step - zero) * RAD_PER_STEP. None if not calibrated yet."""
    if not os.path.exists(path):
        return None
    data = yaml.safe_load(open(path, encoding="utf-8"))
    return {j: (float(data["joints"][j]["zero"]), int(data["joints"][j]["sign"])) for j in JOINTS}


def to_angle(cal, joint, step):
    zero, sign = cal[joint]
    return sign * (step - zero) * RAD_PER_STEP


def to_step(cal, joint, angle):
    zero, sign = cal[joint]
    return zero + sign * angle / RAD_PER_STEP


def find_port():
    """The servo board shows up as a CH340/CH343 (Waveshare / Feetech URT-1) USB serial port."""
    from serial.tools import list_ports
    ports = [p for p in list_ports.comports() if p.vid == 0x1A86]
    return ports[0].device if ports else None


def open_bus(port, mock, cal):
    if mock:
        start = {IDS[j]: to_step(cal, j, REST[j]) for j in JOINTS}
        return FeetechBus(VirtualServos(start))
    port = port or find_port()
    if not port:
        raise SystemExit("SO-100 servo board not found: plug its USB in, or give -p port:=COM5 (Windows) / "
                         "-p port:=/dev/ttyACM0 (Ubuntu)")
    return FeetechBus(port)


class SO100Driver(Node):
    def __init__(self):
        super().__init__("so100_driver")
        p = self.declare_parameter
        self.mock = p("mock", False).value
        self.speed = p("speed", 600).value            # steps/s (600 = ~53 deg/s, gentle)
        self.acc = p("acceleration", 30).value
        self.cal = load_calibration()
        if self.cal is None:
            if not self.mock:
                raise SystemExit("so100_calibration.yaml missing: run so100_calibrate.py first")
            # mock: the middle of each joint range sits at step 2048, as when the arm is assembled the LeRobot way
            self.cal = {j: (2048 - sum(LIMITS[j]) / 2 / RAD_PER_STEP, 1) for j in JOINTS}
        self.bus = open_bus(p("port", "").value, self.mock, self.cal)
        missing = [j for j in JOINTS if not self.bus.ping(IDS[j])]
        if missing:
            raise SystemExit(f"servos not answering: {missing} (IDs {[IDS[j] for j in missing]}) - check power and cables")
        # hold the arm where it is (no jump), then switch the torque on
        here = {j: self.bus.read_position(IDS[j]) for j in JOINTS}
        self.bus.write_goals({IDS[j]: s for j, s in here.items()}, self.speed, self.acc)
        for j in JOINTS:
            self.bus.write(IDS[j], "torque_enable", 1)
        self.state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/arm/joint_command", self.on_command, 10)
        self.create_timer(1 / p("rate", 15.0).value, self.publish_state)
        self.create_timer(2.0, self.check_health)
        # supply voltage at the start: 7.4 V servos run on ~5 V (LeRobot's 5 V supply) or a 2S pack, 12 V servos on 12 V
        self.volts0 = self.bus.read(IDS[JOINTS[0]], "present_voltage") / 10
        angles = ", ".join(f"{j}={to_angle(self.cal, j, s):+.2f}" for j, s in here.items())
        self.get_logger().info(f"SO-100 ready{' (MOCK - no real arm)' if self.mock else ''}, supply {self.volts0:.1f} V, "
                               f"torque on, holding: {angles}")

    def on_command(self, msg):
        goals = {}
        for name, angle in zip(msg.name, msg.position):
            if name in IDS:
                lo, hi = LIMITS[name]
                step = to_step(self.cal, name, min(hi, max(lo, angle)))
                if not 0 <= step < STEPS:
                    self.get_logger().warn(f"{name}: {angle:.2f} rad is outside the servo's 0-4095 range - check the calibration",
                                           throttle_duration_sec=2)
                    continue
                goals[IDS[name]] = step
        if goals:
            try:
                self.bus.write_goals(goals, self.speed, self.acc)
            except BusError as e:
                self.get_logger().error(str(e))

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for j in JOINTS:
            try:
                step, _, load = self.bus.read_state(IDS[j])
                load /= 1000.0
            except BusError as e:
                self.get_logger().warn(str(e), throttle_duration_sec=2)
                return
            msg.name.append(j)
            msg.position.append(to_angle(self.cal, j, step))
            msg.effort.append(load)
        self.state_pub.publish(msg)

    def check_health(self):
        for j in JOINTS:
            try:
                volts = self.bus.read(IDS[j], "present_voltage") / 10
                temp = self.bus.read(IDS[j], "present_temperature")
            except BusError:
                continue
            if temp > 60:
                self.get_logger().warn(f"{j} is {temp} C - let the arm rest", throttle_duration_sec=10)
            if volts < max(4.5, 0.85 * self.volts0):   # sagging battery or a loose power plug
                self.get_logger().warn(f"{j} supply dropped to {volts:.1f} V (was {self.volts0:.1f} V at the start) - "
                                       "check the arm battery / power plug", throttle_duration_sec=10)


def main():
    rclpy.init()
    node = SO100Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("stopped - torque stays ON so the arm does not drop")
        node.bus.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
