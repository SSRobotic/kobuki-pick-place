"""SO-100 arm control: inverse kinematics + pick / place moves, sent on /arm/joint_command.

The kinematic chain is copied from so_arm100.xml (MuJoCo Menagerie), the same numbers as kobuki_x2.urdf,
so the IK target is the point between the jaw pads (TF frame gripper_tip). Targets are in base_link (metres).
Works with fake_arm.py (simulation) and, on demo day, with the real servo driver using the same topics.

Try it by hand while robot_model.launch.py arm:=sim (or the real driver) is running:
    pixi run python robot\\arm_control.py rest            (Windows)      python3 arm_control.py rest   (Ubuntu)
    ... arm_control.py carry | open | close
    ... arm_control.py reach 0.30 0.0 0.30                 (move the jaw centre to x y z in base_link)
    ... arm_control.py joints 0 -1.57 1.57 1.57 0.3        (pan lift elbow wrist gripper, rad: this one = HOME pose)
"""
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
LIMITS = [(-1.92, 1.92), (-3.32, 0.174), (-0.174, 3.14), (-1.66, 1.66)]   # first four joints (so_arm100.xml)
WRIST_ROLL = -math.pi / 2       # held still: jaws open sideways
OPEN, GRIP = 0.8, 0.27          # gripper angle: 74 mm open / about 40 mm = the 4 cm cube
REST = [0.0, -3.32, 3.11, 1.18, -0.17]   # so_arm100.xml "rest" key (folded), gripper closed
CARRY = [0.0, -2.3, 2.2, 1.66, GRIP]     # holds the cube in front, gripper pointing down


def mount_from_urdf():
    """(x, y, z, yaw) of joint so100_mount_joint in kobuki_x2.urdf, so the arm position is set in one place only."""
    try:
        origin = ET.parse(os.path.join(os.path.dirname(os.path.abspath(__file__)), "kobuki_x2.urdf")).getroot().find(
            "joint[@name='so100_mount_joint']/origin")
        x, y, z = (float(v) for v in origin.get("xyz").split())
        return x, y, z, float(origin.get("rpy").split()[2])
    except (OSError, ET.ParseError, AttributeError):
        return 0.06, 0.0, 0.1244, math.pi / 2


MOUNT = mount_from_urdf()


def rot(axis, a):
    c, s = math.cos(a), math.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
            "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
            "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


# (offset from parent, fixed rotation, joint axis) for shoulder_pan .. wrist_roll
CHAIN = [((0, -0.0452, 0.0165), rot("x", math.pi / 2), "y"),
         ((0, 0.1025, 0.0306), rot("x", 1.57079), "x"),
         ((0, 0.11257, 0.028), rot("x", -1.57079), "x"),
         ((0, 0.0052, 0.1349), rot("x", -1.57079), "x"),
         ((0, -0.0601, 0), rot("y", 1.57079), "y")]
TIP = np.array([0.001, -0.085, 0.0])      # jaw centre in the gripper frame (= gripper_tip in the URDF)


def forward(q4):
    """Jaw centre position and pointing direction in base_link for the first four joint angles."""
    r = rot("z", MOUNT[3])
    p = np.array(MOUNT[:3], dtype=float)
    for (off, fixed, axis), a in zip(CHAIN, list(q4) + [WRIST_ROLL]):
        p = p + r @ np.array(off)
        r = r @ fixed @ rot(axis, a)
    return p + r @ TIP, r @ np.array([0.0, -1.0, 0.0])


def inverse(target, q0=None, down_weight=0.003):
    """Joint angles that put the jaw centre on target (base_link). Gripper pointing down is preferred, not forced:
    above shoulder height the wrist limit makes the real arm tilt too. Returns (q4, position error in metres)."""
    target = np.asarray(target, dtype=float)
    q = np.array(q0[:4] if q0 is not None else [0.0, -1.57, 1.57, 1.57], dtype=float)
    # aim the base joint first: the arm reaches along -y of its base, which sits 4.5 cm off the pan axis
    c, s = math.cos(MOUNT[3]), math.sin(MOUNT[3])
    lx, ly = c * (target[0] - MOUNT[0]) + s * (target[1] - MOUNT[1]), -s * (target[0] - MOUNT[0]) + c * (target[1] - MOUNT[1])
    q[0] = math.atan2(lx, -(ly + 0.0452))

    def err(qq):
        p, d = forward(qq)
        return np.concatenate([p - target, down_weight * (d - [0, 0, -1])])
    for _ in range(150):
        e = err(q)
        jac = np.array([(err(q + dq) - e) / 1e-5 for dq in np.eye(4) * 1e-5]).T
        step = np.linalg.solve(jac.T @ jac + 1e-4 * np.eye(4), -jac.T @ e)
        new = np.clip(q + np.clip(step, -0.3, 0.3), [lo for lo, _ in LIMITS], [hi for _, hi in LIMITS])
        if np.abs(new - q).sum() < 1e-6:
            break
        q = new
    return q, float(np.linalg.norm(forward(q)[0] - target))


class ArmController:
    """Sends joint targets and waits for /joint_states to get there. Uses (and spins) an existing rclpy node."""

    def __init__(self, node):
        self.node = node
        self.q = {}
        self.pub = node.create_publisher(JointState, "/arm/joint_command", 10)
        node.create_subscription(JointState, "/joint_states", self.on_joints, 10)

    def on_joints(self, msg):
        self.q.update(zip(msg.name, msg.position))

    def current(self):
        return [self.q.get(n, 0.0) for n in JOINTS[:4]] + [self.q.get("gripper", 0.0)]

    def move(self, q5, timeout=10.0, tol=0.03):
        """q5 = [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, gripper]. True when the arm got there,
        or stopped close to it (real servos sag a little under load, and a gripper closed on the cube stops early)."""
        goal = dict(zip(JOINTS[:4], q5[:4]), wrist_roll=WRIST_ROLL, gripper=q5[4])
        msg = JointState(name=list(goal), position=[float(v) for v in goal.values()])
        start, last, still_since = time.time(), None, None
        while time.time() - start < timeout:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if not self.q:
                continue
            err = max(abs(self.q.get(n, 1e9) - v) for n, v in goal.items())
            if err < tol:
                return True
            now = [self.q.get(n, 0.0) for n in goal]
            still = last is not None and max(abs(a - b) for a, b in zip(now, last)) < 0.002
            still_since = (still_since or time.time()) if still else None
            last = now
            if still_since and time.time() - still_since > 0.6 and time.time() - start > 1.0 and err < 0.15:
                self.node.get_logger().info(f"arm stopped {math.degrees(err):.1f} deg short of the target - accepted")
                return True
        self.node.get_logger().warn("arm did not reach its target in time")
        return False

    def reach(self, target, gripper):
        q, error = inverse(target, self.current())
        if error > 0.015:
            self.node.get_logger().warn(f"target ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) is "
                                        f"{100 * error:.1f} cm out of reach")
            return False
        return self.move(list(q) + [gripper])

    def gripper(self, value):
        return self.move(self.current()[:4] + [value], timeout=4.0)

    def pick(self, cube):
        """cube = centre of the cube in base_link. Ends with the cube lifted 8 cm above the desk;
        back the robot away before stow(), because CARRY holds it at about desk height."""
        cube = np.asarray(cube, dtype=float)
        return (self.gripper(OPEN)
                and self.reach(cube + [0, 0, 0.07], OPEN)      # above it
                and self.reach(cube, OPEN)                     # jaws around it
                and self.gripper(GRIP)                         # close
                and self.reach(cube + [0, 0, 0.08], GRIP))     # lift

    def place(self, spot):
        """spot = centre of the place marker on the desk top, in base_link. Ends with the open gripper 8 cm up."""
        target = np.asarray(spot, dtype=float) + [0, 0, 0.02 + 0.005]   # cube centre just above the desk
        return (self.reach(target + [0, 0, 0.08], GRIP)
                and self.reach(target, GRIP)
                and self.gripper(OPEN)
                and self.reach(target + [0, 0, 0.08], OPEN))

    def stow(self, holding):
        """Fold for driving: CARRY with the cube, REST without."""
        return self.move(CARRY if holding else REST)


def main():
    rclpy.init()
    node = rclpy.create_node("arm_control_cli")
    arm = ArmController(node)
    t = time.time()
    while not arm.q and time.time() - t < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not arm.q:
        print("no /joint_states: start robot_model.launch.py arm:=sim (or the real arm driver) first")
    cmd = sys.argv[1] if len(sys.argv) > 1 else "rest"
    if cmd == "rest":
        ok = arm.move(REST)
    elif cmd == "carry":
        ok = arm.move(CARRY)
    elif cmd in ("open", "close"):
        ok = arm.gripper(OPEN if cmd == "open" else GRIP)
    elif cmd == "joints" and len(sys.argv) == 7:   # shoulder_pan shoulder_lift elbow_flex wrist_flex gripper (rad)
        ok = arm.move([float(v) for v in sys.argv[2:7]])
    elif cmd == "reach" and len(sys.argv) == 5:
        target = [float(v) for v in sys.argv[2:5]]
        q, error = inverse(target, arm.current())
        print("joint angles:", " ".join(f"{n}={v:.3f}" for n, v in zip(JOINTS, q)), f"| error {1000 * error:.1f} mm")
        ok = arm.reach(target, arm.current()[4])
    else:
        print(__doc__)
        ok = False
    print("done" if ok else "failed")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
