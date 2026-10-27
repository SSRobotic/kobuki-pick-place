"""Find the LiDAR mounting yaw (and whether its angles are mirrored) from motion.

Run from the project folder while the LiDAR, Kobuki (--enable-motion)
and teleop nodes run (static TF / slam_toolbox may stay running; not used):
    pixi run python robot\\lidar_calib.py

1. Keep the robot still until it says "Scan A captured".
2. With teleop: drive FORWARD about 30 cm, then turn LEFT about 45 deg, then stop.
3. Wait still until "Scan B captured". Results go to robot\\lidar_calib.txt.

Every hypothesis (yaw 0..358 deg, mirrored or not) projects both scans into the
odom frame with wheel odometry; the right one makes the walls overlap best.
"""
import math
import os
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lidar_calib.txt")
SCANS_PER_SET = 4
STILL_S = 1.0
TIMEOUT_S = 180.0
CELL = 0.05
TOL_SIGMA = 0.04


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Calib(Node):
    def __init__(self):
        super().__init__("lidar_calib")
        self.pose = None
        self.last_motion = time.time()
        self.scans = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)

    def on_odom(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        tw = m.twist.twist
        if abs(tw.linear.x) > 0.005 or abs(tw.angular.z) > 0.01:
            self.last_motion = time.time()

    def on_scan(self, m):
        self.scans.append(m)

    def still(self):
        return self.pose is not None and time.time() - self.last_motion > STILL_S


def scan_points(m):
    pts = []
    for i, r in enumerate(m.ranges):
        if math.isfinite(r) and 0.2 <= r <= 6.0:
            pts.append((m.angle_min + i * m.angle_increment, r))
    return pts


def capture(node, label):
    node.scans.clear()
    pts, pose = [], None
    t0 = time.time()
    while len(pts) < SCANS_PER_SET:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() - t0 > TIMEOUT_S:
            raise SystemExit(f"Timed out waiting for {label}")
        if node.scans:
            m = node.scans.pop(0)
            if node.still():
                pts.append(scan_points(m))
                pose = node.pose
            else:
                pts.clear()
    flat = [p for s in pts for p in s]
    print(f"{label} captured: {len(flat)} points at odom x={pose[0]:.3f} y={pose[1]:.3f} "
          f"yaw={math.degrees(pose[2]):.1f} deg", flush=True)
    return flat, pose


def project(points, pose, yaw, mirror):
    px, py, pth = pose
    out = []
    for a, r in points:
        if mirror:
            a = -a
        th = pth + yaw + a
        out.append((px + r * math.cos(th), py + r * math.sin(th)))
    return out


def score(pa, pb):
    """Mean Gaussian overlap of B points onto their nearest A point (1 = perfect)."""
    grid = {}
    for x, y in pa:
        grid.setdefault((int(math.floor(x / CELL)), int(math.floor(y / CELL))), []).append((x, y))
    reach = int(math.ceil(0.15 / CELL))
    total = 0.0
    for x, y in pb:
        cx, cy = int(math.floor(x / CELL)), int(math.floor(y / CELL))
        best = 0.15 * 0.15
        for dx in range(-reach, reach + 1):
            for dy in range(-reach, reach + 1):
                for qx, qy in grid.get((cx + dx, cy + dy), ()):
                    d2 = (qx - x) ** 2 + (qy - y) ** 2
                    if d2 < best:
                        best = d2
        total += math.exp(-best / (2 * TOL_SIGMA * TOL_SIGMA))
    return total / max(1, len(pb))


def main():
    rclpy.init()
    node = Calib()
    print("Keep the robot still...", flush=True)
    a_pts, a_pose = capture(node, "Scan A")
    print("Now drive FORWARD ~30 cm, then turn LEFT ~45 deg, then STOP and keep still.", flush=True)
    t0 = time.time()
    while True:
        rclpy.spin_once(node, timeout_sec=0.05)
        dx, dy = node.pose[0] - a_pose[0], node.pose[1] - a_pose[1]
        dth = abs((node.pose[2] - a_pose[2] + math.pi) % math.tau - math.pi)
        if math.hypot(dx, dy) > 0.15 and dth > math.radians(20) and node.still():
            break
        if time.time() - t0 > TIMEOUT_S:
            raise SystemExit("Timed out: robot did not move forward >15 cm and turn >20 deg")
    b_pts, b_pose = capture(node, "Scan B")

    moved = math.hypot(b_pose[0] - a_pose[0], b_pose[1] - a_pose[1])
    turned = math.degrees((b_pose[2] - a_pose[2] + math.pi) % math.tau - math.pi)
    results = []
    for mirror in (False, True):
        for yd in range(0, 360, 2):
            y = math.radians(yd)
            s = score(project(a_pts, a_pose, y, mirror), project(b_pts, b_pose, y, mirror))
            results.append((s, yd, mirror))
    results.sort(reverse=True)

    lines = [f"time: {time.strftime('%Y-%m-%d %H:%M:%S')}",
             f"odom motion between scans: {moved:.3f} m, {turned:.1f} deg",
             f"points: A={len(a_pts)} B={len(b_pts)}",
             "top hypotheses (overlap fraction, yaw deg, mirrored):"]
    for s, yd, mirror in results[:8]:
        lines.append(f"  {s:.3f}  yaw={yd:3d}  mirrored={mirror}")
    for want in ((0, False), (180, False)):
        s = next(r[0] for r in results if r[1] == want[0] and r[2] == want[1])
        lines.append(f"current-style check yaw={want[0]} mirrored=False: {s:.3f}")
    best = results[0]
    yaw_deg = best[1] if best[1] <= 180 else best[1] - 360
    lines.append(f"BEST: yaw={yaw_deg} deg ({math.radians(yaw_deg):.4f} rad), mirrored={best[2]}")
    text = "\n".join(lines)
    print(text)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(f"Saved {OUT}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
