"""Real colour camera -> /camera/color/image_raw + /camera/color/camera_info (same topics as fake_camera.py).

Two ways in:
  backend:=openni   ASUS Xtion PRO LIVE through OpenNI2 (needs the OpenNI2 runtime + the `openni` package: Windows `pixi add --pypi openni`, Ubuntu `pip3 install openni`)
  backend:=opencv   any USB webcam (or a video/image file) through OpenCV - the backup if the Xtion will not start
    pixi run python robot\\camera_node.py                                   (Xtion, Windows)
    pixi run python robot\\camera_node.py --ros-args -p backend:=opencv -p device:=1   (webcam number 1)
    pixi run python robot\\camera_node.py --list                            (which webcam numbers work)
    python3 camera_node.py ...                                             (Ubuntu, inside robot/)
The image is tagged with TF frame camera_color_optical_frame (camera_joint in kobuki_x2.urdf: measure it!).
Intrinsics: fx/fy/cx/cy if you know them, otherwise worked out from the horizontal field of view (Xtion = 58 deg).
"""
import math
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def list_cameras(n=6):
    for i in range(n):
        cap = cv2.VideoCapture(i)
        ok, frame = cap.read()
        print(f"device {i}: " + (f"OK {frame.shape[1]}x{frame.shape[0]}" if ok else "-"))
        cap.release()


def patch_openni_for_py314():
    """openni 2.3.0 builds its enum constants inside the ctypes metaclass __new__; Python 3.14 finishes ctypes
    simple types in __init__ instead ("TypeError: abstract class"), so do that step in __init__."""
    if sys.version_info < (3, 14):
        return
    import ctypes
    from openni import utils
    meta, base = utils.CEnumMeta, type(ctypes.c_int)
    if getattr(meta, "_py314_patched", False):
        return

    def new(cls, name, bases, namespace):
        return base.__new__(cls, name, bases, namespace)

    def init(cls2, name, bases, namespace):
        base.__init__(cls2, name, bases, namespace)
        if namespace.get("__module__") != utils.__name__:
            namespace["_values_"].clear()
            for key in list(namespace["_names_"]):
                if not key.startswith("_"):
                    setattr(cls2, key, cls2(namespace[key]))
                    namespace["_values_"][namespace[key]] = key

    meta.__new__, meta.__init__, meta._py314_patched = new, init, True


class OpenNIColor:
    """Xtion colour stream via the `openni` Python package (OpenNI2)."""

    def __init__(self, path, width, height, fps):
        try:
            patch_openni_for_py314()
            from openni import _openni2 as c_api
            from openni import openni2
        except ImportError:
            raise SystemExit("Python package `openni` missing: Windows `pixi add --pypi openni` (in the project folder), "
                             "Ubuntu `pip3 install openni` (and `sudo apt install libopenni2-0`)")
        # Windows: folder with OpenNI2.dll, e.g. C:\Program Files\OpenNI2\Redist (default: $OPENNI2_REDIST64 from the installer)
        openni2.initialize(path) if path else openni2.initialize()
        self.dev = openni2.Device.open_any()
        self.stream = self.dev.create_color_stream()
        self.stream.set_video_mode(c_api.OniVideoMode(pixelFormat=c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888,
                                                      resolutionX=width, resolutionY=height, fps=fps))
        self.stream.set_mirroring_enabled(False)
        self.stream.start()

    def read(self):
        f = self.stream.read_frame()
        rgb = np.frombuffer(f.get_buffer_as_uint8(), dtype=np.uint8).reshape(f.height, f.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


class OpenCVColor:
    def __init__(self, device, width, height):
        src = int(device) if str(device).isdigit() else device
        self.image = cv2.imread(src) if isinstance(src, str) and src.lower().endswith((".png", ".jpg")) else None
        if self.image is None:
            self.cap = cv2.VideoCapture(src)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if not self.cap.isOpened():
                raise SystemExit(f"cannot open camera {device} - try camera_node.py --list")

    def read(self):
        if self.image is not None:          # a still picture, repeated (handy for testing the detector)
            return self.image.copy()
        ok, frame = self.cap.read()
        if not ok:                          # video file at its end: start again (no-op for a webcam)
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ok, frame = self.cap.read()
        return frame if ok else None


class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        p = self.declare_parameter
        backend = p("backend", "openni").value
        w, h, fps = p("width", 640).value, p("height", 480).value, p("fps", 15).value
        self.frame_id = p("frame_id", "camera_color_optical_frame").value
        self.source = (OpenNIColor(p("openni_path", "").value, w, h, 30) if backend == "openni"
                       else OpenCVColor(p("device", "0").value, w, h))
        first = None
        for _ in range(30):
            first = self.source.read()
            if first is not None:
                break
        if first is None:
            raise SystemExit("camera gives no picture")
        self.h, self.w = first.shape[:2]
        fx = p("fx", 0.0).value or (self.w / 2) / math.tan(math.radians(p("hfov_deg", 58.0).value) / 2)
        fy = p("fy", 0.0).value or fx
        cx, cy = p("cx", 0.0).value or (self.w - 1) / 2, p("cy", 0.0).value or (self.h - 1) / 2
        self.info = CameraInfo(width=self.w, height=self.h, distortion_model="plumb_bob", d=[0.0] * 5,
                               k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0], r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                               p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.img_pub = self.create_publisher(Image, "/camera/color/image_raw", 2)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/color/camera_info", 2)
        self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f"{backend} camera {self.w}x{self.h}, fx={fx:.0f} -> /camera/color/image_raw")

    def tick(self):
        img = self.source.read()
        if img is None:
            self.get_logger().warn("no frame", throttle_duration_sec=3)
            return
        msg = Image()
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame_id
        msg.height, msg.width, msg.encoding, msg.step = img.shape[0], img.shape[1], "bgr8", img.shape[1] * 3
        msg.data = np.ascontiguousarray(img).tobytes()
        self.info.header = msg.header
        self.img_pub.publish(msg)
        self.info_pub.publish(self.info)


def main():
    if "--list" in sys.argv:
        list_cameras()
        return
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
