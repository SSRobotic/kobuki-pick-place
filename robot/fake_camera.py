"""Fake ASUS Xtion PRO LIVE colour camera for practising without the robot.

Draws what the camera on the robot (TF frame camera_color_optical_frame from kobuki_x2.urdf) would see
in the fake room of fake_robot.py: floor, walls, furniture, the two station desks, the green cube
(/sim/cube) and the blue place marker. Publishes the same topics a real camera driver does:
  /camera/color/image_raw (bgr8, 640x480, 4 Hz) and /camera/color/camera_info
so cube_detector.py runs unchanged on fake and real images. The arm is not drawn (it never hides anything here).
Needs fake_robot.py and robot_model.launch.py running.
    pixi run python robot\\fake_camera.py      (Windows, from the project folder)
    python3 fake_camera.py                     (Ubuntu, inside robot/)
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from fake_robot import (BOXES_PX, CUBE_SIZE, DESK_TOP, FURNITURE_TOP, MARKER_SIZE, MARKER_XY, WALLS_PX,
                        box_m, to_m)

W, H = 640, 480
FX = FY = 570.0                    # Xtion PRO LIVE colour: 58 x 45 deg field of view at 640 x 480
CX, CY = (W - 1) / 2, (H - 1) / 2
SCALE = 2                          # render at half size, then resize (fast enough for 4 Hz in numpy)
WALL_H = 0.8
COLORS = {  # BGR
    "sky": (205, 205, 205), "floor": (140, 140, 140), "wall": (220, 222, 225),
    "desk_top": (110, 160, 205), "desk_side": (80, 115, 150),
    "furn_top": (150, 135, 125), "furn_side": (115, 100, 95),
    "cube_top": (90, 215, 90), "cube_side": (50, 150, 50), "marker": (190, 95, 30),
}


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rect_sides(x0, y0, x1, y1):
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_camera")
        self.img_pub = self.create_publisher(Image, "/camera/color/image_raw", 2)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 2)
        self.cube = None
        self.create_subscription(PoseStamped, "/sim/cube", lambda m: setattr(self, "cube", m.pose.position), 10)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        u, v = np.meshgrid((np.arange(W // SCALE) * SCALE + SCALE / 2 - 0.5 - CX) / FX,
                           (np.arange(H // SCALE) * SCALE + SCALE / 2 - 0.5 - CY) / FY)
        self.rays = np.stack([u.ravel(), v.ravel(), np.ones(u.size)], axis=1)   # optical frame

        # static scene: (kind, geometry, colour) - kind "h" = horizontal rectangle, "v" = vertical wall piece
        self.static = []
        for x1, y1, x2, y2 in WALLS_PX:
            self.static.append(("v", (*to_m(x1, y1), *to_m(x2, y2), 0.0, WALL_H), "wall"))
        for i, b in enumerate(BOXES_PX):
            x0, y0, x1, y1 = box_m(b)
            top, kind = (DESK_TOP, "desk") if i < 2 else (FURNITURE_TOP, "furn")
            self.static.append(("h", (x0, y0, x1, y1, top), kind + "_top"))
            for seg in rect_sides(x0, y0, x1, y1):
                self.static.append(("v", (*seg, 0.0, top), kind + "_side"))
        mx, my, h = MARKER_XY[0], MARKER_XY[1], MARKER_SIZE / 2
        self.static.append(("h", (mx - h, my - h, mx + h, my + h, DESK_TOP + 0.001), "marker"))

        self.create_timer(0.25, self.render)
        self.get_logger().info("Fake Xtion ready: /camera/color/image_raw (640x480) from TF frame camera_color_optical_frame")

    def render(self):
        try:
            tf = self.tf_buffer.lookup_transform("odom", "camera_color_optical_frame", Time()).transform
        except TransformException:
            return  # robot_model.launch.py not running yet
        o = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        d = self.rays @ quat_matrix(tf.rotation).T
        best = np.full(len(d), np.inf)
        color = np.zeros(len(d), dtype=np.int32)
        names = list(COLORS)

        def hit(t, ok, name):
            ok &= t < best
            best[ok] = t[ok]
            color[ok] = names.index(name)

        with np.errstate(divide="ignore", invalid="ignore"):
            def horizontal(x0, y0, x1, y1, z, name):
                t = (z - o[2]) / d[:, 2]
                px, py = o[0] + t * d[:, 0], o[1] + t * d[:, 1]
                hit(t, (t > 0) & (px >= x0) & (px <= x1) & (py >= y0) & (py <= y1), name)

            def vertical(ax, ay, bx, by, z0, z1, name):
                ex, ey, wx, wy = bx - ax, by - ay, ax - o[0], ay - o[1]
                den = d[:, 0] * ey - d[:, 1] * ex
                t = (wx * ey - wy * ex) / den
                s = (wx * d[:, 1] - wy * d[:, 0]) / den
                z = o[2] + t * d[:, 2]
                hit(t, (t > 0) & (s >= 0) & (s <= 1) & (z >= z0) & (z <= z1), name)

            horizontal(-50, -50, 50, 50, 0.0, "floor")
            for kind, g, name in self.static:
                if np.hypot((g[0] + g[2]) / 2 - o[0], (g[1] + g[3]) / 2 - o[1]) > 7.0:
                    continue  # too far to matter
                (horizontal if kind == "h" else vertical)(*g, name)
            if self.cube is not None:
                c, s = self.cube, CUBE_SIZE / 2
                horizontal(c.x - s, c.y - s, c.x + s, c.y + s, c.z + s, "cube_top")
                for seg in rect_sides(c.x - s, c.y - s, c.x + s, c.y + s):
                    vertical(*seg, c.z - s, c.z + s, "cube_side")

        palette = np.array([COLORS[n] for n in names], dtype=np.float32)
        img = palette[color].reshape(H // SCALE, W // SCALE, 3)
        shade = np.clip(1.15 - 0.06 * np.nan_to_num(best, posinf=8.0), 0.65, 1.0)   # farther = a bit darker
        img = img * shade.reshape(H // SCALE, W // SCALE, 1)
        img = cv2.resize(img, (W, H), interpolation=cv2.INTER_LINEAR)
        img = np.clip(img + np.random.normal(0, 3, img.shape), 0, 255).astype(np.uint8)

        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp, msg.header.frame_id = stamp, "camera_color_optical_frame"
        msg.height, msg.width, msg.encoding, msg.step = H, W, "bgr8", W * 3
        msg.data = img.tobytes()
        info = CameraInfo()
        info.header = msg.header
        info.height, info.width, info.distortion_model = H, W, "plumb_bob"
        info.d = [0.0] * 5
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [FX, 0.0, CX, 0.0, 0.0, FY, CY, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.img_pub.publish(msg)
        self.info_pub.publish(info)


def main():
    rclpy.init()
    node = FakeCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
