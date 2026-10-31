"""Check the camera mount with a tape measure, and fix its tilt/turn in kobuki_x2.urdf.

Put the cube where the camera sees it, measure from the robot centre to the cube centre:
X = forward, Y = to the left (m) - the camera looks out of the front (final build), so X is positive (0.65 m ahead = 0.65).
With camera + detector + robot_model running:
    pixi run python robot\\camera_check.py 0.65 0.0            (Windows)
    python3 camera_check.py 0.65 0.0                           (Ubuntu, inside robot/)
    ... --write      also save the corrected camera_joint into kobuki_x2.urdf (a .bak copy is kept)
It compares where the camera thinks the cube is with your measurement and works out how much the
camera is really tilted / turned. After --write restart robot_model.launch.py and run it again: error < 1 cm = done.
"""
import argparse
import math
import os
import re
import shutil
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kobuki_x2.urdf")
JOINT_RE = re.compile(r'(<joint name="camera_joint".*?<origin xyz="[^"]*" rpy=")([^"]*)(")', re.S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("x", type=float, help="cube centre, metres in front of the robot centre (behind = negative)")
    ap.add_argument("y", type=float, help="metres to the left (right = negative)")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("camera_check")
    buf = Buffer()
    TransformListener(buf, node)
    seen = []
    node.create_subscription(PointStamped, "/detected/cube", lambda m: seen.append(m), 10)
    start = time.time()
    while len(seen) < 8 and time.time() - start < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(seen) < 3:
        raise SystemExit("no /detected/cube - is camera_node.py (or fake_camera.py) and cube_detector.py running, "
                         "and is the cube in the picture?")
    try:
        to_base = buf.lookup_transform("base_link", seen[-1].header.frame_id, Time())
        cam = buf.lookup_transform("base_link", "camera_color_optical_frame", Time()).transform.translation
    except TransformException as e:
        raise SystemExit(f"TF missing ({e}) - is robot_model.launch.py running?")
    q, t = to_base.transform.rotation, to_base.transform.translation
    yaw = 2 * math.atan2(q.z, q.w)
    pts = np.array([[m.point.x, m.point.y, m.point.z] for m in seen])
    px, py, pz = np.median(pts, axis=0)
    det = np.array([math.cos(yaw) * px - math.sin(yaw) * py + t.x, math.sin(yaw) * px + math.cos(yaw) * py + t.y, pz])
    err = math.hypot(det[0] - args.x, det[1] - args.y)
    print(f"camera sees the cube at x={det[0]:.3f} y={det[1]:+.3f}, you measured x={args.x:.3f} y={args.y:+.3f} "
          f"-> off by {100 * err:.1f} cm")

    text = open(URDF, encoding="utf-8").read()
    m = JOINT_RE.search(text)
    roll, pitch, yaw_u = (float(v) for v in m.group(2).split())

    def along(px, py):  # (ahead of the camera, to its left): works whichever way the camera faces
        dx, dy = px - cam.x, py - cam.y
        return math.cos(yaw_u) * dx + math.sin(yaw_u) * dy, -math.sin(yaw_u) * dx + math.cos(yaw_u) * dy
    f_true, l_true = along(args.x, args.y)
    f_det, l_det = along(det[0], det[1])
    # angle below the horizon / to the side, as the camera computed it and as it really is
    drop = cam.z - det[2]
    d_pitch = math.atan2(drop, f_true) - math.atan2(drop, f_det)
    d_yaw = math.atan2(l_true, f_true) - math.atan2(l_det, f_det)
    new = f"{roll:g} {pitch + d_pitch:.3f} {yaw_u + d_yaw:.3f}"
    print(f"camera is tilted {math.degrees(d_pitch):+.1f} deg more down and turned {math.degrees(d_yaw):+.1f} deg "
          f"more left than kobuki_x2.urdf says")
    print(f'camera_joint rpy: "{m.group(2)}"  ->  "{new}"')
    if err < 0.01:
        print("good enough (< 1 cm), nothing to change")
    elif args.write:
        shutil.copy(URDF, URDF + ".bak")
        open(URDF, "w", encoding="utf-8").write(text[:m.start(2)] + new + text[m.end(2):])
        print("saved kobuki_x2.urdf (old one: kobuki_x2.urdf.bak) - restart robot_model.launch.py and check again")
    else:
        print("add --write to save it, then restart robot_model.launch.py and run this again")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
