"""Robot voice: says in Thai what the mission is doing (arrived, found the cube, picked, placed ...).

Listens to /mission/event (published by mission_nav2.py) and plays robot/sounds/<event>.wav through the
laptop speaker, one sentence at a time. Says "หุ่นยนต์พร้อมแล้ว" when it starts = speaker check.
Only talks; it never sends commands to the robot. Sentences: voice_lines.py
    pixi run python robot\\announcer.py          (Windows)
    python3 announcer.py                         (Ubuntu, plays with aplay)
"""
import os
import queue
import subprocess
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from voice_lines import LINES

SOUNDS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sounds")


def play(path):
    if sys.platform == "win32":
        import winsound
        winsound.PlaySound(path, winsound.SND_FILENAME)
    else:
        subprocess.run(["aplay", "-q", path], check=False)


class Announcer(Node):
    def __init__(self):
        super().__init__("announcer")
        self.todo = queue.Queue()
        threading.Thread(target=self.speaker, daemon=True).start()
        self.create_subscription(String, "/mission/event", lambda m: self.say(m.data), 10)
        self.get_logger().info(f"Robot voice on /mission/event ({len(LINES)} sentences in {SOUNDS})")
        self.say("ready")

    def say(self, event):
        path = os.path.join(SOUNDS, event + ".wav")
        if not os.path.exists(path):
            self.get_logger().warn(f"no sound for event '{event}'")
            return
        self.get_logger().info(f"say: {event}")  # event name, not the Thai text: Windows consoles garble it
        self.todo.put(path)

    def speaker(self):
        while True:
            path = self.todo.get()
            if self.todo.qsize() > 2:  # fell behind: skip to the newest news
                continue
            play(path)


def main():
    rclpy.init()
    node = Announcer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
