"""Fake SO-100 servo bus for practising without the arm.

Listens to /arm/joint_command (sensor_msgs/JointState with joint names + target angles) and moves each
joint toward its target at servo-like speed, publishing /joint_states like the real driver will.
Joint names and angles follow kobuki_x2.urdf (= so_arm100.xml): shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_roll, gripper. Starts in the folded "rest" pose.
Started by robot_model.launch.py when you pass arm:=sim.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

REST = {"shoulder_pan": 0.0, "shoulder_lift": -3.32, "elbow_flex": 3.11,
        "wrist_flex": 1.18, "wrist_roll": -1.5708, "gripper": -0.17}
SPEED = 1.2  # rad/s per joint, a bit slower than an STS3215 at full speed


class FakeArm(Node):
    def __init__(self):
        super().__init__("fake_arm")
        self.pos = dict(REST)
        self.goal = dict(REST)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/arm/joint_command", self.on_command, 10)
        self.dt = 1 / 30
        self.create_timer(self.dt, self.tick)
        self.get_logger().info("Fake SO-100 ready: /arm/joint_command -> /joint_states (starts in rest pose)")

    def on_command(self, msg):
        for name, value in zip(msg.name, msg.position):
            if name in self.goal:
                self.goal[name] = value

    def tick(self):
        step = SPEED * self.dt
        for name, goal in self.goal.items():
            self.pos[name] += max(-step, min(step, goal - self.pos[name]))
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.pos)
        msg.position = list(self.pos.values())
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeArm()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
