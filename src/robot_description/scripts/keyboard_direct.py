
#!/usr/bin/env python3

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


HELP = """
========== AGV KEYBOARD CONTROL ==========
 W / Up       : Tien
 S / Down     : Lui
 A / Left     : Xoay trai
 D / Right    : Xoay phai
 Q / E        : Tien va cua trai / phai

 SPACE / K    : Dung
 R / F        : Tang / giam toc do
 Ctrl+C       : Thoat

 Giu phim de duy tri chuyen dong.
 Khong nhan phim -> tu dong dung.
==========================================
"""


class KeyboardDirectNode(Node):

    def __init__(self):
        super().__init__("keyboard_direct_node")

        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.8)

        topic = self.get_parameter("output_topic").value
        self.timeout = float(
            self.get_parameter("command_timeout").value
        )

        if self.timeout <= 0:
            raise ValueError("command_timeout phai > 0")

        self.pub = self.create_publisher(Twist, topic, 10)

        self.lin_speed = 0.25
        self.ang_speed = 1.0

        self.max_linear = 1.0
        self.max_angular = 3.0

        self.v = 0.0
        self.w = 0.0

        self.last_key_time = time.monotonic()

        # Publish Twist lien tuc voi tan so 10 Hz
        self.timer = self.create_timer(0.1, self.publish_command)

        self.get_logger().info(
            f"Keyboard Controller dang publish: {topic}"
        )

    def publish_command(self):
        # Watchdog: khong co phim moi thi tu dung
        if time.monotonic() - self.last_key_time > self.timeout:
            self.v = 0.0
            self.w = 0.0

        msg = Twist()
        msg.linear.x = float(self.v)
        msg.angular.z = float(self.w)

        self.pub.publish(msg)

    def stop(self):
        self.v = 0.0
        self.w = 0.0
        self.publish_command()

    def handle_key(self, key):

        # Dung ngay
        if key in (" ", "k", "K"):
            self.stop()
            print("\n[DUNG]")
            return

        # Tang toc
        if key in ("r", "R"):
            self.lin_speed = min(
                self.max_linear, self.lin_speed + 0.05
            )
            self.ang_speed = min(
                self.max_angular, self.ang_speed + 0.1
            )
            self.show_speed()
            return

        # Giam toc
        if key in ("f", "F"):
            self.lin_speed = max(
                0.05, self.lin_speed - 0.05
            )
            self.ang_speed = max(
                0.2, self.ang_speed - 0.1
            )
            self.show_speed()
            return

        commands = {
            "w": (self.lin_speed, 0.0),
            "s": (-self.lin_speed, 0.0),
            "a": (0.0, self.ang_speed),
            "d": (0.0, -self.ang_speed),
            "q": (self.lin_speed, self.ang_speed * 0.7),
            "e": (self.lin_speed, -self.ang_speed * 0.7),
            "\x1b[A": (self.lin_speed, 0.0),
            "\x1b[B": (-self.lin_speed, 0.0),
            "\x1b[D": (0.0, self.ang_speed),
            "\x1b[C": (0.0, -self.ang_speed),
        }

        lookup = key.lower() if len(key) == 1 else key

        if lookup not in commands:
            return

        self.v, self.w = commands[lookup]

        # Lam moi thoi gian nhan phim
        self.last_key_time = time.monotonic()

        # Gui ngay, khong cho timer
        self.publish_command()

        print(
            f"\rLinear: {self.v:+.2f} m/s | "
            f"Angular: {self.w:+.2f} rad/s    ",
            end="",
            flush=True
        )

    def show_speed(self):
        print(
            f"\n[Toc do] Linear: {self.lin_speed:.2f} m/s | "
            f"Angular: {self.ang_speed:.2f} rad/s"
        )


def get_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.03)

    if not ready:
        return None

    key = sys.stdin.read(1)

    # Nhan dien phim mui ten
    if key == "\x1b":
        for _ in range(2):
            ready, _, _ = select.select(
                [sys.stdin], [], [], 0.02
            )
            if ready:
                key += sys.stdin.read(1)
            else:
                break

    return key


def main():
    if not sys.stdin.isatty():
        print("Chay node nay trong terminal tuong tac.")
        return

    settings = termios.tcgetattr(sys.stdin.fileno())
    node = None

    try:
        rclpy.init()
        node = KeyboardDirectNode()

        print(HELP)

        # Bat che do doc phim, khong echo ra terminal
        tty.setcbreak(sys.stdin.fileno())

        while rclpy.ok():
            key = get_key()

            if key == "\x03":
                break

            if key:
                node.handle_key(key)

            rclpy.spin_once(node, timeout_sec=0.01)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.stop()
            node.destroy_node()

        termios.tcsetattr(
            sys.stdin.fileno(),
            termios.TCSADRAIN,
            settings
        )

        if rclpy.ok():
            rclpy.shutdown()

        print("\nDa thoat Keyboard Controller.")


if __name__ == "__main__":
    main()
