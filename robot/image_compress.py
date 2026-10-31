"""JPEG copy of the camera image for watching it in Foxglove over Wi-Fi.

The raw 640x480 image is ~3.4 MB/s, too much for a phone hotspot; this publishes
/camera/color/image_raw/compressed at most 5 times a second (~0.1 MB/s).
Only for viewing: cube_detector.py and the mission keep using the raw image.
    pixi run python robot\\image_compress.py          (Windows)
    python3 image_compress.py                         (Ubuntu)
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

MAX_RATE = 5.0      # Hz
JPEG_QUALITY = 60


class ImageCompress(Node):
    def __init__(self):
        super().__init__("image_compress")
        self.pub = self.create_publisher(CompressedImage, "/camera/color/image_raw/compressed", 2)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, 2)
        self.last = None
        self.get_logger().info("JPEG copy of /camera/color/image_raw on /camera/color/image_raw/compressed")

    def on_image(self, msg):
        now = self.get_clock().now()
        if self.last is not None and (now - self.last).nanoseconds < 1e9 / MAX_RATE:
            return
        self.last = now
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step // 3, 3)[:, :msg.width]
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, jpg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if ok:
            out = CompressedImage()
            out.header, out.format, out.data = msg.header, "jpeg", jpg.tobytes()
            self.pub.publish(out)


def main():
    rclpy.init()
    node = ImageCompress()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
