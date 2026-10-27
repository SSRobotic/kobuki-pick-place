"""Bring Nav2 nodes up (configure -> activate), in order.

Stand-in for nav2_lifecycle_manager, whose Windows (RoboStack) build crashes on start.
nav2.launch.py runs this automatically on Windows; on Ubuntu the real lifecycle_manager is used.
    python lifecycle_up.py map_server amcl
"""
import sys

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState


def call(node, client, request):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    return future.result()


def bring_up(node, name):
    get = node.create_client(GetState, f"/{name}/get_state")
    change = node.create_client(ChangeState, f"/{name}/change_state")
    while not (get.wait_for_service(timeout_sec=2.0) and change.wait_for_service(timeout_sec=2.0)):
        node.get_logger().info(f"waiting for {name} ...")
    steps = {State.PRIMARY_STATE_UNCONFIGURED: (Transition.TRANSITION_CONFIGURE, "configure"),
             State.PRIMARY_STATE_INACTIVE: (Transition.TRANSITION_ACTIVATE, "activate")}
    while True:
        state = call(node, get, GetState.Request()).current_state.id
        if state == State.PRIMARY_STATE_ACTIVE:
            node.get_logger().info(f"{name} is active")
            return True
        if state not in steps:
            node.get_logger().error(f"{name} is in state {state}, cannot bring it up")
            return False
        tid, label = steps[state]
        node.get_logger().info(f"{label} {name}")
        req = ChangeState.Request()
        req.transition.id = tid
        if not call(node, change, req).success:
            node.get_logger().error(f"{label} {name} failed")
            return False


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
