"""Save where the cube and the place marker are: 4 clicks in RViz -> stations.yaml (mission_nav2.py reads it).

Nav2 + RViz (mission.rviz) running, robot localised (2D Pose Estimate done), cube and marker boxes in place:
    python3 save_stations.py                    (Ubuntu, inside robot/)
    pixi run python robot\\save_stations.py      (Windows)
For each object, in RViz with the Publish Point tool:
  click 1 = on the object, click 2 = on the floor where the robot should come from (the clear side, ~1 m away).
The mission parks PARK m in front of the object on that line, facing it, then drives in to REACH_X.
"""
import datetime
import math
import os
import shutil
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from mission_nav2 import PARK, REACH_X, STATIONS_FILE, park_pose

OBJECTS = [("STATION1", "ลูกบาศก์ (Station 1)"), ("STATION2", "แผ่นจุดวาง (Station 2)")]
ROBOT_RADIUS = 0.18


def blocked_along(m, x, y, heading):
    """Distances (m from the object) on the approach line where the robot would touch a wall of the map."""
    if m is None:
        return []
    grid = np.array(m.data, np.int16).reshape(m.info.height, m.info.width)
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    r = int(math.ceil(ROBOT_RADIUS / res))
    hits = []
    for d in np.arange(REACH_X + 0.05, PARK + 0.01, 0.05):
        cx, cy = x - d * math.cos(heading), y - d * math.sin(heading)
        c, w = int((cx - ox) / res), int((cy - oy) / res)
        patch = grid[max(0, w - r):w + r + 1, max(0, c - r):c + r + 1]
        if patch.size and (patch >= 65).any():
            hits.append(round(float(d), 2))
    return hits


def main():
    rclpy.init()
    node = rclpy.create_node("save_stations")
    clicks, got = [], {"map": None}
    node.create_subscription(PointStamped, "/clicked_point", lambda p: clicks.append(p), 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(OccupancyGrid, "/map", lambda m: got.update(map=m), latched)

    def click(prompt):
        print(prompt, flush=True)
        clicks.clear()
        while not clicks:
            rclpy.spin_once(node, timeout_sec=0.1)
        p = clicks[-1]
        if p.header.frame_id not in ("map", "/map"):
            print(f"  (จุดนี้อยู่ใน frame {p.header.frame_id} ไม่ใช่ map: ตั้ง Fixed Frame ใน RViz เป็น map)")
        print(f"  ได้ x={p.point.x:.2f} y={p.point.y:.2f}", flush=True)
        time.sleep(0.3)
        return p.point.x, p.point.y

    print("ใน RViz กดปุ่ม Publish Point (บนแถบเครื่องมือ) ก่อนคลิกทุกครั้ง\n")
    saved = {}
    for key, label in OBJECTS:
        while True:
            x, y = click(f"[{key}] คลิกตรงกลาง{label}")
            ax, ay = click(f"[{key}] คลิกพื้นฝั่งที่หุ่นจะวิ่งเข้าไปหา{label} (ฝั่งที่โล่ง ห่างประมาณ 1 ม.)")
            if math.hypot(x - ax, y - ay) < 0.4:
                print("  สองจุดใกล้กันเกินไป (< 0.4 ม.): คลิกใหม่ทั้งคู่\n")
                continue
            heading = math.atan2(y - ay, x - ax)
            px, py, pyaw = park_pose(x, y, heading)
            print(f"  หุ่นจะจอดรอที่ x={px:.2f} y={py:.2f} หัน {math.degrees(pyaw):.0f}° แล้ววิ่งเข้าจนห่าง {REACH_X:.2f} ม.")
            hits = blocked_along(got["map"], x, y, heading)
            if hits:
                print(f"  ⚠ ทางเข้าชนกำแพงในแผนที่ที่ระยะ {hits[0]:.2f}–{hits[-1]:.2f} ม. จากวัตถุ: เลือกฝั่งอื่น "
                      f"(ต้องโล่ง {PARK + ROBOT_RADIUS:.2f} ม. จากวัตถุ) · ถ้าเป็นกล่องที่วางวัตถุเองก็ไม่เป็นไร")
                if input("  ใช้ฝั่งนี้เลยไหม? [y = ใช้ / Enter = คลิกใหม่] ").strip().lower() != "y":
                    continue
            saved[key] = [round(x, 3), round(y, 3), round(heading, 4)]
            print()
            break

    if os.path.exists(STATIONS_FILE):
        shutil.copy(STATIONS_FILE, STATIONS_FILE + ".bak")
    with open(STATIONS_FILE, "w", encoding="utf-8") as f:
        f.write(f"# saved by save_stations.py {datetime.datetime.now():%Y-%m-%d %H:%M}: [x, y, heading rad] in the map frame\n")
        for key, v in saved.items():
            f.write(f"{key}: [{v[0]}, {v[1]}, {v[2]}]   # heading {math.degrees(v[2]):.0f} deg\n")
    print(f"เซฟแล้ว: {STATIONS_FILE}")
    print("ซ้อมวิ่ง (ยกหุ่นกลับ Initial ก่อน): python3 mission_nav2.py --nav-only")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
