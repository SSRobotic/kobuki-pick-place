#!/usr/bin/env python3
"""
Final Project mission with REAL navigation (Nav2) and fake arm/detection.

  INIT -> NAV_STATION1 -> DETECT_OBJECT -> PICK -> NAV_STATION2
       -> DETECT_STATION -> PLACE -> RETURN_HOME -> [succeeded]
Same states as final_project_state_machine.py, but the NAV_* states send goals
to Nav2 (nav2_simple_commander) and wait until the robot really arrives.
DETECT_* / PICK / PLACE are still fake (just wait) until the camera and SO-100 code exist.

Needs: robot (fake_robot.py or real drivers) + robot_model.launch.py + nav2.launch.py
    pixi run python robot\\mission_nav2.py      (Windows)
    python3 mission_nav2.py                     (Ubuntu)
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

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

FAKE_WORK_TIME = 2.0   # seconds a fake DETECT / PICK / PLACE takes
MAX_RETRY = 3
NAV_TIMEOUT = 240.0    # seconds allowed for one navigation leg

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


class Init(smach.State):
    def __init__(self, nav):
        super().__init__(outcomes=["succeeded"], output_keys=["retry"])
        self.nav = nav

    def execute(self, ud):
        ud.retry = 0
        self.nav.info("[INIT] robot is at Initial Location - telling AMCL and waiting for Nav2")
        self.nav.setInitialPose(make_pose(self.nav, *GOALS["HOME"]))
        self.nav.waitUntilNav2Active()
        self.nav.info("[INIT] Nav2 is active")
        return "succeeded"


class Navigate(smach.State):
    """Send one Nav2 goal and wait for the result."""

    def __init__(self, nav, name, goal):
        super().__init__(outcomes=["succeeded", "failed", "aborted"],
                         input_keys=["retry"], output_keys=["retry"])
        self.nav, self.name, self.goal = nav, name, goal

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
        ud.retry += 1
        self.nav.warn(f"[{self.name}] navigation {result} ({ud.retry}/{MAX_RETRY})")
        return "aborted" if ud.retry >= MAX_RETRY else "failed"


class FakeTask(smach.State):
    """Placeholder for camera / arm work: waits and succeeds."""

    def __init__(self, nav, name, msg):
        super().__init__(outcomes=["succeeded", "failed", "aborted"],
                         input_keys=["retry"], output_keys=["retry"])
        self.nav, self.name, self.msg = nav, name, msg

    def execute(self, ud):
        self.nav.info(f"[{self.name}] {self.msg} (fake) ...")
        time.sleep(FAKE_WORK_TIME)
        self.nav.info(f"[{self.name}] done")
        return "succeeded"


def main():
    rclpy.init()
    nav = BasicNavigator()

    sm = smach.StateMachine(outcomes=["succeeded", "aborted"])
    tasks = [  # (state name, state object, next state)
        ("NAV_STATION1", Navigate(nav, "NAV_STATION1", "STATION1"), "DETECT_OBJECT"),
        ("DETECT_OBJECT", FakeTask(nav, "DETECT_OBJECT", "detect object on desk"), "PICK"),
        ("PICK", FakeTask(nav, "PICK", "arm picks object"), "NAV_STATION2"),
        ("NAV_STATION2", Navigate(nav, "NAV_STATION2", "STATION2"), "DETECT_STATION"),
        ("DETECT_STATION", FakeTask(nav, "DETECT_STATION", "detect place location"), "PLACE"),
        ("PLACE", FakeTask(nav, "PLACE", "arm places object"), "RETURN_HOME"),
        ("RETURN_HOME", Navigate(nav, "RETURN_HOME", "HOME"), "succeeded"),
    ]
    with sm:
        smach.StateMachine.add("INIT", Init(nav), transitions={"succeeded": "NAV_STATION1"})
        for name, state, nxt in tasks:
            smach.StateMachine.add(name, state,
                                   transitions={"succeeded": nxt, "failed": name, "aborted": "aborted"})

    start = time.time()
    result = sm.execute()
    nav.info(f"Mission finished: {result} in {time.time() - start:.0f} s")
    nav.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
