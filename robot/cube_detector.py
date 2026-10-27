"""Find the coloured cube (Station 1) and the place marker (Station 2) in the camera image.

Works the same on fake_camera.py and on the real Xtion:
  1. threshold the colour in HSV, keep the biggest blob, take its centre pixel
  2. turn that pixel into a ray with the camera intrinsics (/camera/color/camera_info)
  3. intersect the ray with the surface the objects sit on (the floor here: height 0), using TF for the camera pose
This needs no depth image - good, because the Xtion cannot measure depth closer than ~0.8 m.
Publishes geometry_msgs/PointStamped in the odom frame:
  /detected/cube     centre of the cube
  /detected/station  centre of the place marker
and /detector/debug_image (what it saw; add an Image display in RViz).
    pixi run python robot\\cube_detector.py      (Windows, from the project folder)
    python3 cube_detector.py                     (Ubuntu, inside robot/)
Demo day: set the colours of the real cube/marker (hsv_tuner.py finds them), e.g.
    ... cube_detector.py --ros-args -p cube_hsv_low:=[0,120,70] -p cube_hsv_high:=[10,255,255]
and if the objects stand on something instead of the floor: -p surface_height:=0.05
"""
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


def quat_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class CubeDetector(Node):
    def __init__(self):
        super().__init__("cube_detector")
        p = self.declare_parameter
        self.image_topic = p("image_topic", "/camera/color/image_raw").value
        self.frame = p("frame", "odom").value
        self.surface_height = p("surface_height", 0.0).value   # m: what the cube / marker stand on (floor = 0)
        self.cube_size = p("cube_size", 0.04).value
        self.min_area = p("min_area", 30).value
        # OpenCV HSV: H 0-179, S 0-255, V 0-255. Defaults = the green cube / blue marker of the fake room.
        self.targets = {
            "cube": (np.array(p("cube_hsv_low", [45, 100, 60]).value), np.array(p("cube_hsv_high", [85, 255, 255]).value)),
            "station": (np.array(p("marker_hsv_low", [100, 120, 60]).value), np.array(p("marker_hsv_high", [130, 255, 255]).value)),
        }
        self.k = None
        self.pubs = {name: self.create_publisher(PointStamped, f"/detected/{name}", 10) for name in self.targets}
        self.debug_pub = self.create_publisher(Image, "/detector/debug_image", 2)
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.create_subscription(CameraInfo, p("info_topic", "/camera/color/camera_info").value,
                                 lambda m: setattr(self, "k", np.array(m.k).reshape(3, 3)), qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)
        self.last_log = 0.0
        self.get_logger().info(f"Cube detector on {self.image_topic}, objects on a surface {self.surface_height:.2f} m high")

    def on_image(self, msg):
        if self.k is None:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id, Time()).transform
        except TransformException as e:
            self.get_logger().warn(f"no TF {self.frame} <- {msg.header.frame_id}: {e}", throttle_duration_sec=5)
            return
        origin = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        rot = quat_matrix(tf.rotation)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        debug = img.copy()
        found = []
        for name, (lo, hi) in self.targets.items():
            mask = cv2.morphologyEx(cv2.inRange(hsv, lo, hi), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(c) < self.min_area:
                continue
            m = cv2.moments(c)
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            # pixel -> ray in the optical frame -> ray in odom -> hit the plane at the object's centre height
            ray = rot @ np.linalg.solve(self.k, np.array([u, v, 1.0]))
            z = self.surface_height + (self.cube_size / 2 if name == "cube" else 0.0)
            if abs(ray[2]) < 1e-6:
                continue
            t = (z - origin[2]) / ray[2]
            if t <= 0:
                continue
            point = origin + t * ray
            out = PointStamped()
            out.header.stamp, out.header.frame_id = msg.header.stamp, self.frame
            out.point.x, out.point.y, out.point.z = point
            self.pubs[name].publish(out)
            found.append(f"{name} x={point[0]:.3f} y={point[1]:.3f}")
            cv2.drawContours(debug, [c], -1, (0, 0, 255), 2)
            cv2.drawMarker(debug, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(debug, f"{name} ({point[0]:.2f}, {point[1]:.2f})", (int(u) + 12, int(v) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        now = self.get_clock().now().nanoseconds * 1e-9
        if found and now - self.last_log > 3.0:
            self.last_log = now
            self.get_logger().info("seen: " + ", ".join(found))
        dbg = Image()
        dbg.header = msg.header
        dbg.height, dbg.width, dbg.encoding, dbg.step = debug.shape[0], debug.shape[1], "bgr8", debug.shape[1] * 3
        dbg.data = debug.tobytes()
        self.debug_pub.publish(dbg)


def main():
    rclpy.init()
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
