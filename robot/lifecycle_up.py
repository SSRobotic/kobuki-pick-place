"""Bring Nav2 nodes up (configure -> activate), in order.

Stand-in for nav2_lifecycle_manager, whose Windows (RoboStack) build crashes on start.
nav2.launch.py runs this automatically on Windows; on Ubuntu the real lifecycle_manager is used.
    python lifecycle_up.py map_server amcl
"""
import sys

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


TIMEOUT = 5.0       # s to wait for one service answer; a reply can get lost right after discovery
MAX_ATTEMPTS = 5    # configure/activate requests per node before giving up


def call(node, client, request):
    """Service answer, or None if none came within TIMEOUT (never wait forever)."""
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT)
    return future.result() if future.done() else None


def bring_up(node, name):
    get = node.create_client(GetState, f"/{name}/get_state")
    change = node.create_client(ChangeState, f"/{name}/change_state")
    while not (get.wait_for_service(timeout_sec=2.0) and change.wait_for_service(timeout_sec=2.0)):
        node.get_logger().info(f"waiting for {name} ...")
    steps = {State.PRIMARY_STATE_UNCONFIGURED: (Transition.TRANSITION_CONFIGURE, "configure"),
             State.PRIMARY_STATE_INACTIVE: (Transition.TRANSITION_ACTIVATE, "activate")}
    attempts = 0
    while True:
        reply = call(node, get, GetState.Request())
        if reply is None:  # busy in a transition (e.g. waiting for TF) or the reply was lost: ask again
            node.get_logger().info(f"no answer from {name} yet, asking again")
            continue
        state = reply.current_state.id
        if state == State.PRIMARY_STATE_ACTIVE:
            node.get_logger().info(f"{name} is active")
            return True
        if state >= State.TRANSITION_STATE_CONFIGURING:  # configuring / activating ...: wait
            rclpy.spin_once(node, timeout_sec=0.5)
            continue
        if state not in steps or attempts >= MAX_ATTEMPTS:
            node.get_logger().error(f"{name} is in state {reply.current_state.label}, cannot bring it up")
            return False
        tid, label = steps[state]
        attempts += 1
        node.get_logger().info(f"{label} {name}")
        req = ChangeState.Request()
        req.transition.id = tid
        result = call(node, change, req)
        if result is not None and not result.success:
            node.get_logger().warn(f"{label} {name} refused, checking its state again")
        # no answer = still working or reply lost: the next get_state tells which


def main():
    rclpy.init()
    node = rclpy.create_node("lifecycle_up")
    ok = all(bring_up(node, name) for name in sys.argv[1:])
    node.get_logger().info("Managed nodes are active" if ok else "Bring-up FAILED")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
