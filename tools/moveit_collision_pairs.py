"""Find SO-100 link pairs MoveIt should not check, the way the MoveIt Setup Assistant does, using a running move_group.

Run after kobuki_x2.urdf changes (MEASURE values), with robot_model.launch.py + moveit.launch.py running:
    pixi run python tools\\moveit_collision_pairs.py
Prints <disable_collisions> lines: pairs that touch in the arm's real working poses (REST, CARRY, HOME: the meshes
overlap where the real parts only meet) and pairs that touch in almost every random pose. Paste them into
robot/moveit/so100.srdf under <!-- SAMPLED -->, replacing what is there.
"""
import collections
import random
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
LIMITS = [(-1.92, 1.92), (-3.32, 0.174), (-0.174, 3.14), (-1.66, 1.66), (-2.79, 2.79), (-0.174, 1.75)]
POSES = {"rest": [0.0, -3.32, 3.11, 1.18, -1.5708, -0.17],
         "carry": [0.0, -2.3, 2.2, 1.66, -1.5708, 0.27],
         "home": [0.0, -1.57, 1.57, 1.57, -1.5708, 0.8]}
SAMPLES = 3000


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_collision_pairs")
    client = node.create_client(GetStateValidity, "/check_state_validity")
    if not client.wait_for_service(timeout_sec=10.0):
        sys.exit("no /check_state_validity: start moveit.launch.py first")

    def contacts(q):
        req = GetStateValidity.Request(group_name="arm",
                                       robot_state=RobotState(joint_state=JointState(name=ARM, position=list(q))))
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        res = future.result()
        return {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts} if res else set()

    pose_pairs = set()
    for name, q in POSES.items():
        found = contacts(q)
        print(f"{name}: {sorted(found) or 'no contacts'}")
        pose_pairs |= found
    counts = collections.Counter()
    for i in range(SAMPLES):
        counts.update(contacts([random.uniform(lo, hi) for lo, hi in LIMITS]))
    always = {p for p, n in counts.items() if n > 0.95 * SAMPLES}
    print("\ncontacts in random poses (pair: share of samples):")
    for pair, n in counts.most_common():
        print(f"  {pair[0]} - {pair[1]}: {100 * n / SAMPLES:.0f} %")
    print("\n  <!-- SAMPLED by tools/moveit_collision_pairs.py -->")
    for a, b in sorted(pose_pairs | always):
        reason = "Default" if (a, b) in pose_pairs else "Always"
        print(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>')
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
