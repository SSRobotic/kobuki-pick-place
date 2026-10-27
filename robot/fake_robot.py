"""Fake Kobuki + YDLIDAR X2 for practising without the robot.

Replaces window 1 (lidar_ros2.py) and window 2 (kobuki_ros2.py): a virtual robot
drives around the final-project room (floor plan from the project PDF) using
/cmd_vel and publishes the same topics as the real nodes:
  /scan (frame "laser", Best Effort), /odom, TF odom -> base_link,
  plus /stations (MarkerArray) showing Station 1, Station 2 and Initial.
Run robot_model.launch.py, SLAM, RViz and teleop exactly as with the real robot.
For the full mission there is also a green 4 cm cube on the Station 1 desk and a blue place marker on the
Station 2 desk (/sim/cube, drawn in /stations). The cube sticks to the SO-100 jaws when they close around it
(needs robot_model.launch.py arm:=sim) and drops onto the desk when they open. fake_camera.py films all this.
Start somewhere else with: --ros-args -p start_pose:=[5.04,1.32,0.0]

    python3 fake_robot.py                      (Ubuntu)
    pixi run python robot\\fake_robot.py        (Windows, from the project folder)
"""
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
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

# Objects for the pick & place practice. Heights are guesses: measure the real desks on demo day.
DESK_TOP = 0.22                 # m, top of both station desks (BOXES_PX[0] and BOXES_PX[1])
FURNITURE_TOP = 0.45            # m, the other boxes (only the camera cares)
CUBE_SIZE = 0.04                # m, the coloured cube to pick
CUBE_START = (5.50, 1.32)       # on the Station 1 desk, 7 cm from its edge
MARKER_XY = (2.85, -3.12)       # blue square on the Station 2 desk = where to place the cube
MARKER_SIZE = 0.10
GRIP_CLOSED = 0.35              # gripper joint below this = closed (rad, so_arm100.xml "Jaw")
GRIP_OPEN = 0.5                 # above this = open
GRAB_DISTANCE = 0.03            # jaw centre must be this close to the cube centre to catch it


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


def box_m(box_px):
    """(left, top, right, bottom) plan pixels -> (x0, y0, x1, y1) metres in odom."""
    (xa, ya), (xb, yb) = to_m(box_px[0], box_px[1]), to_m(box_px[2], box_px[3])
    return min(xa, xb), min(ya, yb), max(xa, xb), max(ya, yb)


def surface_height(x, y):
    """Height of whatever the cube would land on at (x, y)."""
    for i, b in enumerate(BOXES_PX):
        x0, y0, x1, y1 = box_m(b)
        if x0 <= x <= x1 and y0 <= y <= y1:
            return DESK_TOP if i < 2 else FURNITURE_TOP
    return 0.0


class FakeRobot(Node):
    def __init__(self):
        super().__init__("fake_robot")
        self.segs = build_segments()
        # start somewhere else to practise one step, e.g. at Station 1: --ros-args -p start_pose:=[5.04,1.32,0.0]
        self.x, self.y, self.yaw = self.declare_parameter("start_pose", [0.0, 0.0, 0.0]).value
        self.v = self.w = 0.0
        self.last_cmd = self.get_clock().now()
        self.blocked = False

        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/stations", 1)
        self.cube_pub = self.create_publisher(PoseStamped, "/sim/cube", 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 1)

        # the cube starts a little off the nominal spot each run, so the camera really has to find it
        jitter = self.declare_parameter("cube_jitter", 0.03).value
        self.cube = [CUBE_START[0] + random.uniform(-jitter, jitter), CUBE_START[1] + random.uniform(-jitter, jitter),
                     DESK_TOP + CUBE_SIZE / 2]
        self.held = False
        self.gripper = None
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, "/joint_states", self.on_joints, 10)

        self.dt = 0.02
        self.create_timer(self.dt, self.step)            # odom 50 Hz like the Kobuki
        self.create_timer(0.25, self.publish_scan)       # 4 Hz like the X2
        self.create_timer(0.05, self.update_cube)
        self.create_timer(0.2, self.publish_stations)
        self.get_logger().info("Fake robot ready at Initial Location. Publishing /scan, /odom, TF odom -> base_link, /stations, /sim/cube")
        for name, (px, py) in STATIONS_PX.items():
            x, y = to_m(px, py)
            self.get_logger().info(f"  {name}: x={x:.2f} y={y:.2f} (odom)")
        self.get_logger().info(f"  Cube on Station 1 desk: x={self.cube[0]:.3f} y={self.cube[1]:.3f} | "
                               f"place marker on Station 2 desk: x={MARKER_XY[0]:.2f} y={MARKER_XY[1]:.2f}")

    def on_joints(self, msg):
        if "gripper" in msg.name:
            self.gripper = msg.position[msg.name.index("gripper")]

    def update_cube(self):
        """The cube sticks to the jaws when they close around it and drops onto whatever is below when they open."""
        if self.gripper is not None:
            try:
                t = self.tf_buffer.lookup_transform("odom", "gripper_tip", Time()).transform.translation
                tip = (t.x, t.y, t.z)
            except TransformException:
                tip = None
            if tip and not self.held and self.gripper < GRIP_CLOSED and math.dist(tip, self.cube) < GRAB_DISTANCE:
                self.held = True
                self.get_logger().info(f"Cube grabbed (jaw centre {100 * math.dist(tip, self.cube):.1f} cm from cube centre)")
            elif self.held and self.gripper > GRIP_OPEN:
                self.held = False
                self.cube[2] = surface_height(self.cube[0], self.cube[1]) + CUBE_SIZE / 2
                err = math.hypot(self.cube[0] - MARKER_XY[0], self.cube[1] - MARKER_XY[1])
                where = "on the Station 2 marker" if err < MARKER_SIZE / 2 else f"{100 * err:.0f} cm from the marker"
                self.get_logger().info(f"Cube released at x={self.cube[0]:.3f} y={self.cube[1]:.3f} z={self.cube[2]:.3f} - {where}")
            if self.held and tip:
                self.cube = list(tip)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.cube
        msg.pose.orientation.w = 1.0
        self.cube_pub.publish(msg)

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
        for i, (pos, size, rgb) in enumerate((
                (self.cube, (CUBE_SIZE,) * 3, (0.2, 0.75, 0.25)),
                ((*MARKER_XY, DESK_TOP + 0.001), (MARKER_SIZE, MARKER_SIZE, 0.002), (0.1, 0.35, 0.85)))):
            m = Marker()
            m.header.frame_id = "odom"
            m.ns, m.id, m.type = "objects", i, Marker.CUBE
            m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = size
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 1.0
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
