"""Snapshot the running ROS 2 graph for mapping diagnostics.

Run from the project folder:
    pixi run python robot\\ros_check.py
Writes robot\\ros_check.txt (and prints the same text).
"""
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
import tf2_ros

LISTEN_S = 6.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ros_check.txt")


class Check(Node):
    def __init__(self):
        super().__init__("ros_check")
        self.scans, self.odoms, self.maps = [], [], []
        self.create_subscription(LaserScan, "/scan", lambda m: self.scans.append((time.time(), m)), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", lambda m: self.odoms.append((time.time(), m)), 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "/map", lambda m: self.maps.append((time.time(), m)), map_qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    lines = []
    p = lines.append
    rclpy.init()
    n = Check()
    t0 = time.time()
    while time.time() - t0 < LISTEN_S:
        rclpy.spin_once(n, timeout_sec=0.1)

    p(f"time: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    p("== nodes ==")
    for name, ns in sorted(n.get_node_names_and_namespaces()):
        p(f"  {ns.rstrip('/')}/{name}")
    p("== topics ==")
    for t, types in sorted(n.get_topic_names_and_types()):
        pubs = n.count_publishers(t)
        subs = n.count_subscribers(t)
        p(f"  {t}  [{', '.join(types)}]  pubs={pubs} subs={subs}")

    p(f"== rates over {LISTEN_S:.0f}s ==")
    for label, arr in (("/scan", n.scans), ("/odom", n.odoms), ("/map", n.maps)):
        p(f"  {label}: {len(arr)} msgs, {len(arr) / LISTEN_S:.1f} Hz")

    if n.scans:
        m = n.scans[-1][1]
        rs = list(m.ranges)
        valid = [(i, r) for i, r in enumerate(rs) if math.isfinite(r) and m.range_min <= r <= m.range_max]
        p("== last scan ==")
        p(f"  frame={m.header.frame_id} n={len(rs)} valid={len(valid)} angle_min={math.degrees(m.angle_min):.1f} "
          f"angle_max={math.degrees(m.angle_max):.1f} range=[{m.range_min},{m.range_max}]")
        if valid:
            i, r = min(valid, key=lambda v: v[1])
            ang = math.degrees(m.angle_min + i * m.angle_increment)
            p(f"  closest raw return: {r:.2f} m at {ang:.1f} deg in laser frame (may be the robot itself)")
            far = [(j, rr) for j, rr in valid if rr >= 0.20]
            if far:
                i, r = min(far, key=lambda v: v[1])
                ang = math.degrees(m.angle_min + i * m.angle_increment)
                try:
                    tr = n.tf_buffer.lookup_transform("base_link", m.header.frame_id, Time())
                    off = math.degrees(yaw_of(tr.transform.rotation))
                    base_ang = (ang + off + 180) % 360 - 180
                    p(f"  CLOSEST object >=0.20 m: {r:.2f} m at {ang:.1f} deg laser = {base_ang:.1f} deg ROBOT "
                      f"(0=robot front, +90=robot left, 180=robot back)")
                except Exception as e:  # noqa: BLE001
                    p(f"  CLOSEST object >=0.20 m: {r:.2f} m at {ang:.1f} deg laser (no TF: {type(e).__name__})")
            for sector, (a0, a1) in (("front(-20..20)", (-20, 20)), ("left(70..110)", (70, 110)),
                                     ("back(160..200)", (160, 200)), ("right(-110..-70)", (-110, -70))):
                vals = []
                for j, rr in valid:
                    a = math.degrees(m.angle_min + j * m.angle_increment)
                    a = (a + 180) % 360 - 180
                    lo, hi = a0, a1
                    if hi > 180:
                        if a >= lo or a <= hi - 360:
                            vals.append(rr)
                    elif lo <= a <= hi:
                        vals.append(rr)
                p(f"  {sector}: min {min(vals):.2f} m" if vals else f"  {sector}: no returns")

    if n.odoms:
        o = n.odoms[-1][1]
        pp = o.pose.pose
        p("== last odom ==")
        p(f"  frame={o.header.frame_id}->{o.child_frame_id} x={pp.position.x:.3f} y={pp.position.y:.3f} "
          f"yaw={math.degrees(yaw_of(pp.orientation)):.1f} deg")

    if n.maps:
        g = n.maps[-1][1]
        d = g.info
        known = sum(1 for v in g.data if v >= 0)
        occ = sum(1 for v in g.data if v >= 65)
        p("== last map ==")
        p(f"  {d.width}x{d.height} res={d.resolution:.3f} known_cells={known} occupied={occ}")

    p("== TF ==")
    try:
        frames = n.tf_buffer.all_frames_as_string()
        p("  " + frames.strip().replace("\n", "\n  ") if frames.strip() else "  (no frames)")
    except Exception as e:  # noqa: BLE001
        p(f"  frames error: {e}")
    for parent, child in (("map", "odom"), ("odom", "base_link"), ("base_link", "laser"), ("map", "base_link")):
        try:
            tr = n.tf_buffer.lookup_transform(parent, child, Time())
            t = tr.transform.translation
            p(f"  {parent}->{child}: x={t.x:.3f} y={t.y:.3f} z={t.z:.3f} yaw={math.degrees(yaw_of(tr.transform.rotation)):.1f} deg")
        except Exception as e:  # noqa: BLE001
            p(f"  {parent}->{child}: MISSING ({type(e).__name__})")

    text = "\n".join(lines)
    print(text)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(f"\nSaved {OUT}")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
