"""SO-100 arm control: inverse kinematics + pick / place moves, sent on /arm/joint_command.

The kinematic chain is copied from so_arm100.xml (MuJoCo Menagerie), the same numbers as kobuki_x2.urdf,
so the IK target is the point between the jaw pads (TF frame gripper_tip). Targets are in base_link (metres).
Works with fake_arm.py (simulation) and, on demo day, with the real servo driver using the same topics.

Try it by hand while robot_model.launch.py arm:=sim (or the real driver) is running:
    pixi run python robot\\arm_control.py rest            (Windows)      python3 arm_control.py rest   (Ubuntu)
    ... arm_control.py carry | home | open | close
    ... arm_control.py reach 0.30 0.0 0.30                 (move the jaw centre to x y z in base_link)
    ... arm_control.py joints 0 -1.57 1.57 1.57 0.3        (pan lift elbow wrist gripper, rad: this one = HOME pose)
    ... arm_control.py pick 0.31 0.0 0.11                  (the mission's pick: cube centre, here a 4 cm cube on a 9 cm box)
    ... arm_control.py place 0.31 0.0 0.09                 (the mission's place: marker on top of a 9 cm box)
Add --moveit (with moveit.launch.py running) and every move is planned by MoveIt around the robot body and played by
so100_trajectory_bridge.py, instead of sending the target angles straight to the servos:
    ... arm_control.py --moveit reach 0.30 0.0 0.30
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
MOVEIT_LIMITS = dict(zip(JOINTS, LIMITS + [(-2.79, 2.79), (-0.174, 1.75)]))   # all six, as in kobuki_x2.urdf
WRIST_ROLL = -math.pi / 2       # held still: jaws open sideways
# gripper angle: 0.8 = 74 mm open. The real jaw touches the 4 cm cube at 0.20 rad (measured 31 Oct; the model's
# 0.27 never touched it and the cube fell out): GRIP aims 0.1 rad past that so the jaw really squeezes
OPEN, GRIP = 0.8, 0.10
try:   # so100_calibrate_app.py measures it with the cube between the jaws and saves it with the calibration
    import yaml
    with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "so100_calibration.yaml"), encoding="utf-8") as f:
        GRIP = float((yaml.safe_load(f) or {}).get("grip", GRIP))
except (OSError, ValueError, ImportError):
    pass
REST = [0.0, -3.32, 3.11, 1.18, -0.17]   # so_arm100.xml "rest" key (folded), gripper closed
CARRY = [0.0, -2.3, 2.2, 1.66, GRIP]     # holds the cube in front, gripper pointing down
HOME = [0.0, -1.57, 1.57, 1.57, OPEN]    # the calibration HOME pose, jaws open
# the real fingertips reach 4.5 cm below gripper_tip (measured 31 Oct, cube on a 9 cm box): aim the jaw centre this
# far above the cube centre, so the tips stop 0.5 cm above what the cube stands on and the cube sits between the fingers
GRASP_UP = 0.03


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
    """Sends joint targets and waits for /joint_states to get there. Uses (and spins) an existing rclpy node.
    moveit=True: MoveIt's move_group plans each move (collision-free, timed) and executes it; IK stays here,
    MoveIt gets joint goals (KDL cannot solve a 6-D pose for a 5-joint arm)."""

    def __init__(self, node, moveit=False):
        self.node = node
        self.q = {}
        self.pub = node.create_publisher(JointState, "/arm/joint_command", 10)
        node.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.move_group = None
        if moveit:
            from moveit_msgs.action import MoveGroup
            from rclpy.action import ActionClient
            self.move_group = ActionClient(node, MoveGroup, "/move_action")
            if not self.move_group.wait_for_server(timeout_sec=15.0):
                raise RuntimeError("no MoveIt /move_action: start moveit.launch.py first")
            node.get_logger().info("arm moves are planned by MoveIt")

    def on_joints(self, msg):
        self.q.update(zip(msg.name, msg.position))

    def current(self):
        return [self.q.get(n, 0.0) for n in JOINTS[:4]] + [self.q.get("gripper", 0.0)]

    def move(self, q5, timeout=10.0, tol=0.03):
        """q5 = [shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, gripper]. True when the arm got there,
        or stopped close to it (real servos sag a little under load, and a gripper closed on the cube stops early)."""
        if self.move_group:
            return self.plan_and_move(q5, timeout)
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

    def plan_and_move(self, q5, timeout):
        """MoveIt version of move(): the arm group first, then the gripper group, each only if it has to move."""
        now = self.current() + [self.q.get("wrist_roll", WRIST_ROLL)]
        arm_goal = list(q5[:4]) + [WRIST_ROLL]
        ok = True
        # 0.02 rad: real servos read a step or two off, and that alone must not start a move
        if max(abs(a - b) for a, b in zip(arm_goal, now[:4] + now[5:])) > 0.02:
            ok = self.moveit_goal("arm", JOINTS[:5], arm_goal, timeout)
        if ok and abs(q5[4] - now[4]) > 0.02:
            ok = self.moveit_goal("gripper", ["gripper"], [q5[4]], timeout)
        return ok

    def moveit_goal(self, group, names, values, timeout):
        """Plan and execute one joint goal with move_group. True on success."""
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = group
        req.num_planning_attempts = 3
        req.allowed_planning_time = 3.0
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.5
        req.start_state.is_diff = True   # start from the current /joint_states
        # MoveIt refuses a goal outside the URDF limits; the real arm can read a little past them (REST ~ -3.36)
        values = [min(hi - 0.005, max(lo + 0.005, v)) for v, (lo, hi) in zip(values, (MOVEIT_LIMITS[n] for n in names))]
        req.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(v), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for n, v in zip(names, values)])]
        goal.planning_options.plan_only = False
        sent = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, sent, timeout_sec=5.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().warn(f"MoveIt did not take the {group} goal")
            return False
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, done, timeout_sec=timeout + 10.0)
        if done.result() is None:
            handle.cancel_goal_async()
            self.node.get_logger().warn(f"MoveIt {group} move timed out")
            return False
        code = done.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.node.get_logger().warn(f"MoveIt {group} move failed, error code {code} "
                                        "(-1 no plan, -4 control failed, -10/-12 start/goal in collision)")
            return False
        return True

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
        grasp = np.asarray(cube, dtype=float) + [0, 0, GRASP_UP]
        return (self.gripper(OPEN)
                and self.reach(grasp + [0, 0, 0.07], OPEN)     # above it
                and self.reach(grasp, OPEN)                    # jaws around it
                and self.gripper(GRIP)                         # close
                and self.reach(grasp + [0, 0, 0.08], GRIP))    # lift

    def place(self, spot):
        """spot = centre of the place marker on the desk top, in base_link. Ends with the open gripper 8 cm up."""
        target = np.asarray(spot, dtype=float) + [0, 0, 0.02 + 0.005 + GRASP_UP]   # cube centre just above the desk
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
    use_moveit = "--moveit" in sys.argv
    if use_moveit:
        sys.argv.remove("--moveit")
    arm = ArmController(node, moveit=use_moveit)
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
    elif cmd == "home":   # upper arm straight up, forearm out (front), gripper pointing down (calibration HOME)
        ok = arm.move(HOME)
    elif cmd in ("open", "close"):
        ok = arm.gripper(OPEN if cmd == "open" else GRIP)
    elif cmd == "joints" and len(sys.argv) == 7:   # shoulder_pan shoulder_lift elbow_flex wrist_flex gripper (rad)
        ok = arm.move([float(v) for v in sys.argv[2:7]])
    elif cmd in ("pick", "place") and len(sys.argv) == 5:   # the mission's own moves: pick = cube centre, place = marker
        target = [float(v) for v in sys.argv[2:5]]
        ok = arm.pick(target) if cmd == "pick" else arm.place(target)
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
