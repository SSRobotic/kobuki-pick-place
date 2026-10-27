"""Fake Kobuki + YDLIDAR X2 for practising without the robot.

Replaces window 1 (lidar_ros2.py) and window 2 (kobuki_ros2.py): a virtual robot
drives around the final-project room (floor plan from the project PDF) using
/cmd_vel and publishes the same topics as the real nodes:
  /scan (frame "laser", Best Effort), /odom, TF odom -> base_link,
  plus /stations (MarkerArray) showing Station 1, Station 2 and Initial.
Run robot_model.launch.py, SLAM, RViz and teleop exactly as with the real robot.

    python3 fake_robot.py                      (Ubuntu)
    pixi run python robot\\fake_robot.py        (Windows, from the project folder)
"""
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

PX = 0.015               # metres per pixel of the floor plan picture in the PDF
START_PX = (425, 940)    # "Initial Location" = odom origin; robot starts facing Station 1 (up)
ROBOT_RADIUS = 0.18      # Kobuki is ~0.35 m wide
LASER_YAW = -math.pi / 2  # same as joint laser_joint in kobuki_x2.urdf
MAX_V, MAX_W = 0.12, 0.6  # same limits as kobuki_ros2.py --enable-motion
CMD_TIMEOUT = 0.4

WALLS_PX = [  # (x1, y1, x2, y2) line segments in plan pixels
    (278, 537, 676, 537), (676, 537, 676, 1041), (676, 1041, 278, 1041), (278, 1041, 278, 537),
    (556, 537, 556, 597), (556, 655, 556, 912), (556, 970, 556, 1041),  # right wall with two doors
    (278, 783, 405, 783), (488, 783, 556, 783),                          # middle wall with doorway
]
BOXES_PX = [  # (left, top, right, bottom) desks and furniture
    (298, 540, 386, 578),   # desk at Station 1
    (626, 729, 670, 769),   # desk at Station 2
    (366, 666, 464, 710), (505, 679, 549, 777), (288, 797, 386, 841), (494, 803, 553, 847),
    (557, 847, 616, 891), (288, 891, 332, 989), (366, 994, 484, 1038),
]
STATIONS_PX = {"Station 1": (337, 604), "Station 2": (597, 750), "Initial": (425, 940)}


def to_m(px, py):
    """Plan pixel -> metres in odom. +x = up on the plan (robot front at start), +y = left."""
    return (START_PX[1] - py) * PX, (START_PX[0] - px) * PX


def build_segments():
    segs = [to_m(x1, y1) + to_m(x2, y2) for x1, y1, x2, y2 in WALLS_PX]
    for l, t, r, b in BOXES_PX:
        corners = [(l, t), (r, t), (r, b), (l, b)]
        for i in range(4):
            segs.append(to_m(*corners[i]) + to_m(*corners[(i + 1) % 4]))
    return np.array(segs, dtype=float)


def raycast(segs, ox, oy, angles, max_range):
    """Distance from (ox, oy) along each world angle to the nearest segment (inf if none)."""
    dx, dy = np.cos(angles)[:, None], np.sin(angles)[:, None]
    x1, y1, x2, y2 = segs.T
    ex, ey = x2 - x1, y2 - y1
    wx, wy = x1 - ox, y1 - oy
    with np.errstate(divide="ignore", invalid="ignore"):
        denom = dx * ey - dy * ex
        t = (wx * ey - wy * ex) / denom
        u = (wx * dy - wy * dx) / denom
    hit = (denom != 0) & (t > 0) & (u >= 0) & (u <= 1)
    dist = np.where(hit, t, np.inf).min(axis=1)
    dist[dist > max_range] = np.inf
    return dist


def clearance(segs, x, y):
    """Shortest distance from point (x, y) to any segment."""
    x1, y1, x2, y2 = segs.T
    ex, ey = x2 - x1, y2 - y1
    s = np.clip(((x - x1) * ex + (y - y1) * ey) / (ex * ex + ey * ey), 0.0, 1.0)
    return float(np.hypot(x1 + s * ex - x, y1 + s * ey - y).min())


def yaw_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.segs = build_segments()
        self.x = self.y = self.yaw = 0.0
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        self.blocked = False

        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/stations", 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 1)

        self.dt = 0.02
        self.create_timer(self.dt, self.step)            # odom 50 Hz like the Kobuki
        self.create_timer(0.25, self.publish_scan)       # 4 Hz like the X2
        self.create_timer(1.0, self.publish_stations)
        self.get_logger().info("Fake robot ready at Initial Location. Publishing /scan, /odom, TF odom -> base_link, /stations")
        for name, (px, py) in STATIONS_PX.items():
            x, y = to_m(px, py)
            self.get_logger().info(f"  {name}: x={x:.2f} y={y:.2f} (odom)")

    def on_cmd(self, msg):
        self.v = max(-MAX_V, min(MAX_V, msg.linear.x))
        self.w = max(-MAX_W, min(MAX_W, msg.angular.z))
        self.last_cmd = self.get_clock().now()

    def step(self):
        now = self.get_clock().now()
        if (now - self.last_cmd).nanoseconds * 1e-9 > CMD_TIMEOUT:
            self.v = self.w = 0.0
        yaw = self.yaw + self.w * self.dt
        x = self.x + self.v * math.cos(yaw) * self.dt
        y = self.y + self.v * math.sin(yaw) * self.dt
        if clearance(self.segs, x, y) >= ROBOT_RADIUS:
            self.x, self.y = x, y
            self.blocked = False
        elif not self.blocked:
            self.blocked = True
            self.get_logger().warn("Bump! Wall or furniture ahead - turn away or reverse")
        self.yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        stamp = now.to_msg()
        q = yaw_quaternion(self.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = self.v if not self.blocked else 0.0
        odom.twist.twist.angular.z = self.w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = "base_link"
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf.sendTransform(t)

    def publish_scan(self):
        n = 720
        inc = 2 * math.pi / n
        angles = -math.pi + inc * np.arange(n)                 # laser frame, counter-clockwise
        ranges = raycast(self.segs, self.x, self.y, angles + self.yaw + LASER_YAW, 8.0)
        ranges = ranges + np.random.normal(0.0, 0.01, n)       # ~1 cm noise
        ranges[np.random.random(n) < 0.05] = np.inf            # a few missing returns, like the X2
        ranges[ranges < 0.12] = np.inf

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser"
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi - inc, inc
        scan.scan_time = 0.25
        scan.range_min, scan.range_max = 0.12, 8.0
        scan.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(scan)

    def publish_stations(self):
        markers = MarkerArray()
        colors = {"Station 1": (0.9, 0.1, 0.1), "Station 2": (0.9, 0.1, 0.1), "Initial": (0.1, 0.5, 1.0)}
        for i, (name, (px, py)) in enumerate(STATIONS_PX.items()):
            x, y = to_m(px, py)
            for kind, z, mtype in (("disc", 0.01, Marker.CYLINDER), ("label", 0.5, Marker.TEXT_VIEW_FACING)):
                m = Marker()
                m.header.frame_id = "odom"
                m.ns, m.id, m.type = kind, i, mtype
                m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
                m.pose.orientation.w = 1.0
                if mtype == Marker.CYLINDER:
                    m.scale.x, m.scale.y, m.scale.z = 0.4, 0.4, 0.02
                else:
                    m.text = name
                    m.scale.z = 0.25
                m.color.r, m.color.g, m.color.b = colors[name]
                m.color.a = 0.8
                markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = FakeRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
