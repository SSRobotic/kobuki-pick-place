"""Find the colour numbers for cube_detector.py: click on the cube (or the place marker) in the camera image.

    pixi run python robot\\hsv_tuner.py                       (live: /camera/color/image_raw, needs camera_node.py)
    pixi run python robot\\hsv_tuner.py --image photo.png     (a saved picture)
    python3 hsv_tuner.py ...                                 (Ubuntu, inside robot/)
In the window: left-click the cube -> its colour range is picked and the white mask shows what will be detected.
Keys: 1 = I am clicking the CUBE, 2 = I am clicking the PLACE MARKER, s = save a picture, q = quit.
When you quit it prints the -p ... line to paste after cube_detector.py --ros-args.
Fine-tune with the sliders if the mask has holes (lower S/V min) or picks up other things (raise them).
"""
import argparse

import cv2
import numpy as np


def range_from_patch(hsv, x, y, r=4):
    """HSV low/high around a clicked pixel: hue +-10, saturation and value with room for shadows."""
    patch = hsv[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1].reshape(-1, 3).astype(int)
    h, s, v = np.median(patch, axis=0)
    lo = [max(0, int(h) - 10), max(40, int(s * 0.5)), max(40, int(v * 0.4))]
    hi = [min(179, int(h) + 10), 255, 255]
    return lo, hi


def mask_of(bgr, lo, hi):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.morphologyEx(cv2.inRange(hsv, np.array(lo), np.array(hi)), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def side_by_side(bgr, lo, hi, label):
    mask = cv2.cvtColor(mask_of(bgr, lo, hi), cv2.COLOR_GRAY2BGR)
    out = np.hstack([bgr, mask])
    cv2.putText(out, f"{label}: low {lo} high {hi}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    return out


def ros_args(ranges):
    names = {"cube": "cube_hsv", "station": "marker_hsv"}
    return " ".join(f"-p {names[k]}_low:=[{','.join(map(str, lo))}] -p {names[k]}_high:=[{','.join(map(str, hi))}]"
                    for k, (lo, hi) in ranges.items())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", help="use a saved picture instead of the camera topic")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--click", nargs=2, type=int, metavar=("X", "Y"), help="no window: pick at X Y, save a picture, quit")
    ap.add_argument("--save", default="hsv_tuner.png")
    args = ap.parse_args()

    frame = {"img": cv2.imread(args.image) if args.image else None}
    node = None
    if not args.image:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        rclpy.init()
        node = rclpy.create_node("hsv_tuner")

        def on_image(m):
            img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
            frame["img"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else img.copy()
        node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)  # also gets best-effort cameras (openni2_camera)

    ranges, target = {}, ["cube"]
    if args.click:                      # headless mode (used to make the guide's pictures)
        while frame["img"] is None:
            import rclpy
            rclpy.spin_once(node, timeout_sec=0.1)
        hsv = cv2.cvtColor(frame["img"], cv2.COLOR_BGR2HSV)
        ranges["cube"] = range_from_patch(hsv, *args.click)
        cv2.imwrite(args.save, side_by_side(frame["img"], *ranges["cube"], "cube"))
        print(ros_args(ranges))
        return

    win = "hsv_tuner (1 = cube, 2 = marker, s = save, q = quit)"
    cv2.namedWindow(win)
    for name, val in (("H min", 0), ("H max", 179), ("S min", 40), ("V min", 40)):
        cv2.createTrackbar(name, win, val, 179 if name.startswith("H") else 255, lambda _: None)

    def on_mouse(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and frame["img"] is not None:
            lo, hi = range_from_patch(cv2.cvtColor(frame["img"], cv2.COLOR_BGR2HSV), x % frame["img"].shape[1], y)
            for name, v in (("H min", lo[0]), ("H max", hi[0]), ("S min", lo[1]), ("V min", lo[2])):
                cv2.setTrackbarPos(name, win, v)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        if node is not None:
            import rclpy
            rclpy.spin_once(node, timeout_sec=0.01)
        if frame["img"] is None:
            cv2.waitKey(30)
            continue
        lo = [cv2.getTrackbarPos("H min", win), cv2.getTrackbarPos("S min", win), cv2.getTrackbarPos("V min", win)]
        hi = [cv2.getTrackbarPos("H max", win), 255, 255]
        ranges[target[0]] = (lo, hi)
        view = side_by_side(frame["img"], lo, hi, target[0])
        cv2.imshow(win, view)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("1"):
            target[0] = "cube"
        elif key == ord("2"):
            target[0] = "station"
        elif key == ord("s"):
            cv2.imwrite(args.save, view)
            print("saved", args.save)
        elif key in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    print("\npaste after cube_detector.py --ros-args :\n" + ros_args(ranges))


if __name__ == "__main__":
    main()
