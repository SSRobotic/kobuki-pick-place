#!/usr/bin/env python3
"""
Final Project mission: Nav2 driving + camera detection + SO-100 pick and place.

  INIT -> NAV_STATION1 -> DETECT_OBJECT -> PICK -> NAV_STATION2
       -> DETECT_STATION -> PLACE -> RETURN_HOME -> [succeeded]
Same states as final_project_state_machine.py:
  NAV_*          send a goal to Nav2 and wait until the robot really arrives
  DETECT_*       wait for cube_detector.py to see the cube / the place marker (position in odom)
  PICK / PLACE   turn to face it, drive straight until it is 0.30 m ahead (arm reach), look again,
                 move the arm with arm_control.py, back away 0.2 m, fold the arm
A failed PICK goes back to DETECT_OBJECT (and PLACE to DETECT_STATION); 3 failures in a row = aborted.

Needs, each in its own window (simulation; on demo day the real drivers replace the fake_* nodes):
  fake_robot.py, robot_model.launch.py arm:=sim, fake_camera.py, cube_detector.py, nav2.launch.py
    pixi run python robot\\mission_nav2.py      (Windows)
    python3 mission_nav2.py                     (Ubuntu)
Driving only (no camera, no arm needed): add --nav-only
"""
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

from arm_control import ArmController

try:
    import smach
except ImportError:  # RoboStack on Windows has no smach: use a tiny stand-in with the same API
    class _UserData:
        pass

    class _State:
        def __init__(self, outcomes, input_keys=(), output_keys=()):
            self.outcomes = outcomes

    class _StateMachine:
        def __init__(self, outcomes):
            self.outcomes, self.states, self.userdata = outcomes, {}, _UserData()
            self.first = None

        def __enter__(self):
            _StateMachine.current = self
            return self

        def __exit__(self, *exc):
            return False

        @staticmethod
        def add(name, state, transitions):
            sm = _StateMachine.current
            sm.first = sm.first or name
            sm.states[name] = (state, transitions)

        def execute(self):
            name = self.first
            while name not in self.outcomes:
                state, transitions = self.states[name]
                name = transitions[state.execute(self.userdata)]
            return name

    class smach:  # noqa: N801  (mimics the smach module)
        State, StateMachine = _State, _StateMachine

MAX_RETRY = 3
NAV_TIMEOUT = 240.0    # seconds allowed for one navigation leg
DETECT_TIMEOUT = 8.0   # seconds to wait for the camera to see the cube / marker
REACH_X = 0.30         # m ahead of the robot centre where the arm works best (arm_control.py IK)
BACK_OFF = 0.20        # m to reverse after picking / placing, so the arm can fold clear of the desk

# Goal poses in the map frame (x, y, yaw). The map was made starting at Initial Location,
# so Initial = (0, 0). Yaw makes the robot face the desk where the arm will work.
GOALS = {
    "STATION1": (5.04, 1.32, 0.0),            # desk is in front (+x)
    "STATION2": (2.85, -2.58, -math.pi / 2),  # desk is to the right (-y)
    "HOME": (0.0, 0.0, 0.0),
}


def make_pose(nav, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"  # stamp left at 0 = "use the latest TF"
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


class Robot:
    """Everything the states share: Nav2, odometry, detections, the arm and straight-line driving."""

    def __init__(self, nav):
        self.nav = nav
        self.arm = ArmController(nav)
        self.odom = None
        self.seen = {"cube": [], "station": []}   # (receive time, xyz in odom)
        self.target = {}                          # last good detection per name
        self.cmd = nav.create_publisher(Twist, "/cmd_vel", 10)
        nav.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m.pose.pose), 10)
        for name in self.seen:
            nav.create_subscription(PointStamped, f"/detected/{name}",
                                    lambda m, n=name: self.seen[n].append((time.time(), (m.point.x, m.point.y, m.point.z))), 10)

    def spin(self, sec=0.05):
        rclpy.spin_once(self.nav, timeout_sec=sec)

    def pose(self):
        o = self.odom
        return o.position.x, o.position.y, 2 * math.atan2(o.orientation.z, o.orientation.w)

    def to_base(self, p):
        """odom point -> base_link (the fake/real robot drives on a flat floor, so this is a 2D turn)."""
        x, y, yaw = self.pose()
        dx, dy = p[0] - x, p[1] - y
        return np.array([math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy, p[2]])

    def look(self, name, timeout=DETECT_TIMEOUT, n=5):
        """Median of n fresh detections, or None."""
        start = time.time()
        while time.time() - start < timeout:
            self.spin()
            fresh = [p for t, p in self.seen[name] if t > start]
            if len(fresh) >= n:
                self.seen[name] = self.seen[name][-50:]
                return np.median(np.array(fresh), axis=0)
        return None

    def drive(self, distance, speed=0.08, timeout=20.0):
        """Straight line on odometry (Nav2 is idle between goals, so /cmd_vel is ours)."""
        while self.odom is None:
            self.spin()
        x0, y0, _ = self.pose()
        start, last_move, last_d = time.time(), time.time(), 0.0
        while time.time() - start < timeout:
            self.spin(0.02)
            x, y, _ = self.pose()
            done = math.hypot(x - x0, y - y0)
            if abs(distance) - done < 0.003:
                break
            if done - last_d > 0.002:
                last_move, last_d = time.time(), done
            elif time.time() - last_move > 2.0:
                self.nav.warn("blocked while driving straight")
                break
            v = max(0.025, min(speed, 0.8 * (abs(distance) - done)))
            self.send(math.copysign(v, distance), 0.0)
        self.send(0.0, 0.0)

    def turn(self, angle, timeout=15.0):
        while self.odom is None:
            self.spin()
        goal = self.pose()[2] + angle
        start = time.time()
        while time.time() - start < timeout:
            self.spin(0.02)
            err = math.atan2(math.sin(goal - self.pose()[2]), math.cos(goal - self.pose()[2]))
            if abs(err) < 0.01:
                break
            self.send(0.0, max(-0.4, min(0.4, 1.5 * err + math.copysign(0.05, err))))
        self.send(0.0, 0.0)

    def send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd.publish(t)

    def approach(self, name):
        """Face the target and stop when it is REACH_X ahead. Returns its fresh position in base_link, or None."""
        b = self.to_base(self.target[name])
        if abs(math.atan2(b[1], b[0])) > 0.03:
            self.turn(math.atan2(b[1], b[0]))
        b = self.to_base(self.target[name])
        move = max(-0.3, min(0.5, b[0] - REACH_X))
        self.nav.info(f"  approach: {name} is {b[0]:.2f} m ahead, driving {move:+.2f} m")
        self.drive(move)
        again = self.look(name, timeout=4.0)       # closer view = better position
        if again is not None:
            self.target[name] = again
        return self.to_base(self.target[name])


class Init(smach.State):
    def __init__(self, robot):
        super().__init__(outcomes=["succeeded"], output_keys=["retry"])
        self.robot = robot

    def execute(self, ud):
        nav = self.robot.nav
        ud.retry = 0
        nav.info("[INIT] robot is at Initial Location - telling AMCL and waiting for Nav2")
        nav.setInitialPose(make_pose(nav, *GOALS["HOME"]))
        nav.waitUntilNav2Active()
        nav.info("[INIT] Nav2 is active - folding the arm")
        self.robot.arm.stow(holding=False)
        return "succeeded"


class Navigate(smach.State):
    """Send one Nav2 goal and wait for the result."""

    def __init__(self, robot, name, goal):
        super().__init__(outcomes=["succeeded", "failed", "aborted"],
                         input_keys=["retry"], output_keys=["retry"])
        self.nav, self.name, self.goal = robot.nav, name, goal

    def execute(self, ud):
        x, y, yaw = GOALS[self.goal]
        self.nav.info(f"[{self.name}] go to {self.goal} x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg")
        start = time.time()
        self.nav.goToPose(make_pose(self.nav, x, y, yaw))
        last_print = 0.0
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb and time.time() - last_print > 5.0:
                last_print = time.time()
                self.nav.info(f"[{self.name}] {fb.distance_remaining:.2f} m to go, "
                              f"{fb.number_of_recoveries} recoveries")
            if time.time() - start > NAV_TIMEOUT:
                self.nav.cancelTask()
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav.info(f"[{self.name}] arrived in {time.time() - start:.0f} s")
            ud.retry = 0
            return "succeeded"
        return fail(self.nav, self.name, f"navigation {result}", ud)


def fail(nav, name, why, ud):
    ud.retry += 1
    nav.warn(f"[{name}] {why} ({ud.retry}/{MAX_RETRY})")
    return "aborted" if ud.retry >= MAX_RETRY else "failed"


class Detect(smach.State):
    """Wait until the camera reports the cube / marker and remember where it is (odom)."""

    def __init__(self, robot, name, what):
        super().__init__(outcomes=["succeeded", "failed", "aborted"],
                         input_keys=["retry"], output_keys=["retry"])
        self.robot, self.name, self.what = robot, name, what

    def execute(self, ud):
        r = self.robot
        r.nav.info(f"[{self.name}] looking for the {self.what} ...")
        p = r.look(self.what)
        if p is None:
            r.turn(0.15 if ud.retry % 2 == 0 else -0.3)   # glance sideways before the next try
            return fail(r.nav, self.name, f"no {self.what} in the camera image", ud)
        r.target[self.what] = p
        b = r.to_base(p)
        r.nav.info(f"[{self.name}] {self.what} at x={p[0]:.3f} y={p[1]:.3f} (odom) = "
                   f"{b[0]:.2f} m ahead, {b[1]:+.2f} m left of the robot")
        ud.retry = 0
        return "succeeded"


class Manipulate(smach.State):
    """PICK or PLACE: approach, move the arm, back off, fold."""

    def __init__(self, robot, name, what, picking):
        super().__init__(outcomes=["succeeded", "failed", "aborted"],
                         input_keys=["retry"], output_keys=["retry"])
        self.robot, self.name, self.what, self.picking = robot, name, what, picking

    def execute(self, ud):
        r = self.robot
        b = r.approach(self.what)
        r.nav.info(f"[{self.name}] {self.what} at {b[0]:.3f} {b[1]:+.3f} {b[2]:.3f} (base_link) - moving the arm")
        ok = r.arm.pick(b) if self.picking else r.arm.place(b)
        r.drive(-BACK_OFF)
        r.arm.stow(holding=self.picking and ok)
        if not ok:
            return fail(r.nav, self.name, "arm could not finish", ud)
        r.nav.info(f"[{self.name}] done")
        ud.retry = 0
        return "succeeded"


def main():
    rclpy.init()
    nav = BasicNavigator()
    robot = Robot(nav)

    sm = smach.StateMachine(outcomes=["succeeded", "aborted"])
    tasks = [  # (state name, state object, next state, state to go back to when it fails)
        ("NAV_STATION1", Navigate(robot, "NAV_STATION1", "STATION1"), "DETECT_OBJECT", "NAV_STATION1"),
        ("DETECT_OBJECT", Detect(robot, "DETECT_OBJECT", "cube"), "PICK", "DETECT_OBJECT"),
        ("PICK", Manipulate(robot, "PICK", "cube", picking=True), "NAV_STATION2", "DETECT_OBJECT"),
        ("NAV_STATION2", Navigate(robot, "NAV_STATION2", "STATION2"), "DETECT_STATION", "NAV_STATION2"),
        ("DETECT_STATION", Detect(robot, "DETECT_STATION", "station"), "PLACE", "DETECT_STATION"),
        ("PLACE", Manipulate(robot, "PLACE", "station", picking=False), "RETURN_HOME", "DETECT_STATION"),
        ("RETURN_HOME", Navigate(robot, "RETURN_HOME", "HOME"), "succeeded", "RETURN_HOME"),
    ]
    if "--nav-only" in sys.argv:  # driving practice without camera and arm: Station 1 -> Station 2 -> home
        tasks = [(name, state, {"DETECT_OBJECT": "NAV_STATION2", "DETECT_STATION": "RETURN_HOME"}.get(nxt, nxt), again)
                 for name, state, nxt, again in tasks if name.startswith(("NAV", "RETURN"))]
    with sm:
        smach.StateMachine.add("INIT", Init(robot), transitions={"succeeded": "NAV_STATION1"})
        for name, state, nxt, again in tasks:
            smach.StateMachine.add(name, state,
                                   transitions={"succeeded": nxt, "failed": again, "aborted": "aborted"})

    start = time.time()
    result = sm.execute()
    nav.info(f"Mission finished: {result} in {time.time() - start:.0f} s")
    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
