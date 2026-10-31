"""MoveIt -> SO-100: plays the trajectories move_group plans on /arm/joint_command.

Serves control_msgs/FollowJointTrajectory (the actions named in moveit/moveit_controllers.yaml):
    /arm_controller/follow_joint_trajectory       shoulder_pan .. wrist_roll
    /gripper_controller/follow_joint_trajectory   gripper
and sends the goals on the topic arm_control.py uses, so the same MoveIt setup drives the real arm (so100_driver.py)
and the fake one (robot_model.launch.py arm:=sim). Started by moveit.launch.py.

The STS3215 servos run their own smooth speed profile to every goal they get. Streaming MoveIt's dense trajectory
(30 small goals a second) made them brake and start again on every step - a jerky, rattling arm (31 Oct). So the
servos get few, far-apart goals: only the end of the path when MoveIt planned a straight line in joint space (most
moves), else waypoints at least SPACING apart, each handed over before the servo starts braking for it.
"""
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

RATE = 30.0       # Hz, how often the joint error is checked
GOAL_TOL = 0.05   # rad: every joint this close to the last point = done
STRAIGHT = 0.05   # rad: a path that never leaves the straight line by more than this = one goal
SPACING = 0.3     # rad: smallest step between waypoints of a bent path
HANDOVER = 0.15   # rad: send the next waypoint when the servos are this close to the current one
RESEND = 0.5      # s: repeat the current goal now and then (a lost message must not stall the arm)
MIN_SPEED = 0.4   # rad/s: slowest the servos are expected to move, for the time limit
SETTLE = 4.0      # s extra for the servos to settle at the end


def waypoints(points):
    """Goals for the servos (lists of positions) from a JointTrajectory's points."""
    first, last = list(points[0].positions), list(points[-1].positions)
    line = [b - a for a, b in zip(first, last)]
    length2 = sum(d * d for d in line) or 1e-12

    def off_line(p):
        t = max(0.0, min(1.0, sum((x - a) * d for x, a, d in zip(p, first, line)) / length2))
        return max(abs(x - (a + t * d)) for x, a, d in zip(p, first, line))

    if all(off_line(p.positions) < STRAIGHT for p in points):
        return [last]
    goals, ref = [], first
    for p in points[1:-1]:
        if max(abs(a - b) for a, b in zip(p.positions, ref)) >= SPACING:
            goals.append(list(p.positions))
            ref = p.positions
    return goals + [last]


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__("so100_trajectory_bridge")
        group = ReentrantCallbackGroup()
        self.q = {}
        self.busy = threading.Lock()   # one trajectory at a time on the servo bus
        self.pub = self.create_publisher(JointState, "/arm/joint_command", 10)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10, callback_group=group)
        self.servers = [ActionServer(self, FollowJointTrajectory, f"/{name}/follow_joint_trajectory", self.execute,
                                     goal_callback=lambda _: GoalResponse.ACCEPT,
                                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
                        for name in ("arm_controller", "gripper_controller")]
        self.get_logger().info("MoveIt trajectory bridge ready: follow_joint_trajectory -> /arm/joint_command")

    def on_joints(self, msg):
        self.q.update(zip(msg.name, msg.position))

    def send(self, names, positions):
        msg = JointState(name=list(names), position=[float(v) for v in positions])
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        names, points = list(traj.joint_names), traj.points
        result = FollowJointTrajectory.Result()
        if not points:
            goal_handle.succeed()
            return result
        goals = waypoints(points)
        path = sum(max(abs(a - b) for a, b in zip(g0, g1)) for g0, g1 in zip([list(points[0].positions)] + goals, goals))
        planned = points[-1].time_from_start.sec + 1e-9 * points[-1].time_from_start.nanosec
        deadline = time.monotonic() + max(planned, path / MIN_SPEED) + SETTLE
        with self.busy:
            for i, goal in enumerate(goals):
                final = i == len(goals) - 1
                sent, last, still_since = 0.0, None, None
                while True:
                    if goal_handle.is_cancel_requested:
                        self.send(names, [self.q.get(n, 0.0) for n in names])   # stop where it is
                        goal_handle.canceled()
                        return result
                    if time.monotonic() - sent > RESEND:
                        self.send(names, goal)
                        sent = time.monotonic()
                    now = [self.q.get(n, 1e9) for n in names]
                    err = max(abs(a - b) for a, b in zip(now, goal))
                    if not final and err < HANDOVER:
                        break
                    if final:
                        if err < GOAL_TOL:
                            break
                        # a gripper closed on the cube stops short of its target: "stopped close" counts as there
                        still = last is not None and max(abs(a - b) for a, b in zip(now, last)) < 0.002
                        still_since = (still_since or time.monotonic()) if still else None
                        last = now
                        if still_since and time.monotonic() - still_since > 0.5 and err < 0.15:
                            self.get_logger().info(f"{names[0]}..: stopped {err:.2f} rad short of the target - accepted")
                            break
                    if time.monotonic() > deadline:
                        self.get_logger().warn(f"{', '.join(names)} did not reach the end of the trajectory")
                        result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                        goal_handle.abort()
                        return result
                    time.sleep(1.0 / RATE)
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        goal_handle.succeed()
        return result


def main():
    rclpy.init()
    node = TrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
