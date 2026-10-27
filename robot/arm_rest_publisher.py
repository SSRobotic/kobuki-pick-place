"""Publish the SO-100 joints in the folded "rest" pose on /joint_states until a real arm driver exists.

robot_state_publisher needs a value for every moving joint before it can send the arm's TF; without this
RViz shows the arm links as "No transform". Started by robot_model.launch.py (turn it off with arm:=none
once a real SO-100 node publishes /joint_states).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# so_arm100.xml "rest" key, wrist rolled -90 deg, gripper closed
REST = {"shoulder_pan": 0.0, "shoulder_lift": -3.32, "elbow_flex": 3.11,
        "wrist_flex": 1.18, "wrist_roll": -1.5708, "gripper": -0.17}


class ArmRest(Node):
    def __init__(self):
        super().__init__("arm_rest_publisher")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info("Publishing SO-100 rest pose on /joint_states (no real arm driver yet)")

    def tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(REST)
        msg.position = list(REST.values())
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ArmRest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
