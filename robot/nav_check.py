"""Why does Nav2 drive the robot the wrong way? Run this while Nav2 runs (robot standing still) and read the ❌ lines.

    python3 nav_check.py                       (Ubuntu, inside robot/)
    pixi run python robot\\nav_check.py         (Windows)

Checks, in order:
  1. slam_toolbox still running next to AMCL  -> two programs move the map, the robot pose jumps
  2. /scan and /odom arrive, and how late (a slow VM makes them late)
  3. AMCL knows where the robot is (2D Pose Estimate given, small spread)
  4. the red LiDAR dots lie on the black walls of the map (pose right, laser TF right)
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

LISTEN_S = 6.0


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wall_match(grid, info, xs, ys):
    """Fraction of map-frame points that land on (or next to) an occupied map cell."""
    res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
    col = np.floor((xs - ox) / res).astype(int)
    row = np.floor((ys - oy) / res).astype(int)
    inside = (col >= 1) & (row >= 1) & (col < info.width - 1) & (row < info.height - 1)
    if inside.sum() < 20:
        return 0.0
    col, row = col[inside], row[inside]
    occ = grid >= 65
    hit = np.zeros(col.shape, bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            hit |= occ[row + dr, col + dc]
    return float(hit.mean())


def main():
    rclpy.init()
    node = rclpy.create_node("nav_check")
    tf = Buffer()
    TransformListener(tf, node)
    got = {"scan": [], "odom": [], "map": None, "amcl": None}
    node.create_subscription(LaserScan, "/scan", lambda m: got["scan"].append((time.time(), m)), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", lambda m: got["odom"].append((time.time(), m)), 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.update(map=m), latched)
    for qos in (latched, 10):   # AMCL's amcl_pose QoS differs between versions: listen both ways
        node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: got.update(amcl=m), qos)

    t0 = time.time()
    while time.time() - t0 < LISTEN_S:
        rclpy.spin_once(node, timeout_sec=0.05)
    bad = 0

    def ok(text):
        print("✅ " + text)

    def fail(text):
        nonlocal bad
        bad += 1
        print("❌ " + text)

    names = node.get_node_names()
    if "slam_toolbox" in names and "amcl" in names:
        fail("slam_toolbox ยังเปิดอยู่พร้อม AMCL: ปิดหน้าต่าง slam_toolbox (Ctrl+C) แล้วเปิด Nav2 ใหม่")
    elif "amcl" not in names:
        fail("ไม่เจอ AMCL: เปิด Nav2 (nav2.launch.py) ก่อน")
    else:
        ok("AMCL ทำงาน ไม่มี slam_toolbox แย่ง")

    now_ros = node.get_clock().now().nanoseconds * 1e-9
    for topic, want in (("scan", 3.0), ("odom", 10.0)):
        msgs = got[topic]
        if len(msgs) < 2:
            fail(f"/{topic} ไม่มีข้อมูล: เปิดหน้าต่าง {'1 LiDAR' if topic == 'scan' else '2 Kobuki'}")
            continue
        hz = (len(msgs) - 1) / (msgs[-1][0] - msgs[0][0])
        st = msgs[-1][1].header.stamp
        late = now_ros - (st.sec + st.nanosec * 1e-9)
        text = f"/{topic} {hz:.1f} Hz, ช้า {late:.2f} s"
        if hz < want * 0.6:
            fail(text + f" (ควรได้ ≥ {want * 0.6:.0f} Hz: เครื่อง/VM ช้า ปิด RViz ตัวที่ไม่ใช้ เพิ่ม CPU ให้ VM)")
        elif abs(late) > 0.5:
            fail(text + " (เวลาเพี้ยน: VM ช้าหรือนาฬิกา VM ไม่ตรง)")
        else:
            ok(text)

    a = got["amcl"]
    if a is None:
        fail("AMCL ยังไม่บอกตำแหน่งหุ่น: ใน RViz กด 2D Pose Estimate คลิกตรงหุ่นจริง ลากไปทางที่หุ่นหัน")
    else:
        c = a.pose.covariance
        spread, turn = math.sqrt(max(c[0], c[7])), math.degrees(math.sqrt(max(c[35], 0.0)))
        p = a.pose.pose
        text = f"AMCL: หุ่นอยู่ x={p.position.x:.2f} y={p.position.y:.2f} หัน {math.degrees(yaw_of(p.orientation)):.0f}° (คลาด ±{spread:.2f} m, ±{turn:.0f}°)"
        if spread > 0.25 or turn > 20:
            fail(text + " → ยังไม่แน่ใจตำแหน่ง: กด 2D Pose Estimate ให้ตรงหุ่นจริง")
        else:
            ok(text)

    m, scans = got["map"], got["scan"]
    if m is None:
        fail("ไม่ได้รับ /map: เช็ค map:=... ของ nav2.launch.py ว่าชี้ไฟล์ .yaml ที่เซฟไว้")
    elif scans:
        scan = scans[-1][1]
        try:
            t = tf.lookup_transform("map", scan.header.frame_id, Time()).transform
        except TransformException as e:
            fail(f"ไม่มี TF map -> {scan.header.frame_id}: {e} (AMCL ยังไม่ได้ตำแหน่ง หรือไม่ได้เปิดโมเดลหุ่น หน้าต่าง 3)")
        else:
            r = np.array(scan.ranges, float)
            ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
            keep = np.isfinite(r) & (r > 0.2) & (r < 6.0)
            r, ang = r[keep], ang[keep]
            grid = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
            lx, ly, lyaw = t.translation.x, t.translation.y, yaw_of(t.rotation)

            def match(extra_yaw=0.0, mirror=False):
                a2 = (-ang if mirror else ang) + lyaw + extra_yaw
                return wall_match(grid, m.info, lx + r * np.cos(a2), ly + r * np.sin(a2))

            now = match()
            others = {f"หมุน {d}°" + (" + กลับด้าน" if mi else ""): match(math.radians(d), mi)
                      for d in (0, 90, 180, 270) for mi in (False, True) if d or mi}
            best = max(others, key=others.get)
            text = f"จุด LiDAR ทับกำแพงในแผนที่ {now * 100:.0f}%"
            if now >= 0.5:
                ok(text)
            elif others[best] >= 0.5 and others[best] > now + 0.2:
                fail(text + f" แต่ถ้า{best}จะทับ {others[best] * 100:.0f}% → TF ของ LiDAR อาจผิด: "
                     "กด 2D Pose Estimate ก่อน ถ้ายังเป็น รัน lidar_calib.py")
            else:
                fail(text + " → ตำแหน่งหุ่นผิด: กด 2D Pose Estimate ให้ตรงหุ่นจริง (หรือแผนที่ไม่ใช่ห้องนี้)")

    print("\nผ่านหมด: สั่ง 2D Goal Pose ได้" if not bad else f"\nแก้ ❌ {bad} ข้อ แล้วรันใหม่")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
