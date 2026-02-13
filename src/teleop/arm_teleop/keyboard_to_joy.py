import select
import sys
import termios
import tty
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


KEY_BINDINGS = {
    "a": ("axis", 0, -1.0),  # left stick left
    "d": ("axis", 0, 1.0),   # left stick right
    "w": ("axis", 1, 1.0),   # left stick up
    "s": ("axis", 1, -1.0),  # left stick down
    "j": ("axis", 3, -1.0),  # right stick left
    "l": ("axis", 3, 1.0),   # right stick right
    "i": ("axis", 4, 1.0),   # right stick up
    "k": ("axis", 4, -1.0),  # right stick down
    "u": ("button", 4, 1),   # LB
    "o": ("button", 5, 1),   # RB
    " ": ("button", 0, 1),   # A
    "m": ("button", 1, 1),   # B
    "n": ("button", 3, 1),   # Y
}


class KeyboardToJoy(Node):
    def __init__(self) -> None:
        super().__init__("keyboard_to_joy")
        self.declare_parameter("axis_count", 8)
        self.declare_parameter("button_count", 11)
        self.declare_parameter("repeat_rate_hz", 20.0)

        self.axis_count = int(self.get_parameter("axis_count").value)
        self.button_count = int(self.get_parameter("button_count").value)
        rate = float(self.get_parameter("repeat_rate_hz").value)

        self.publisher = self.create_publisher(Joy, "joy", 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)

        # Try to grab a TTY for keypresses (works when launched via ros2 launch).
        self._tty = sys.stdin
        if not sys.stdin.isatty():
            try:
                self._tty = open("/dev/tty")
            except OSError as exc:
                self.get_logger().error(f"KeyboardToJoy needs a TTY; failed to open /dev/tty: {exc}")
                raise

        self._stdin_settings = termios.tcgetattr(self._tty)
        tty.setcbreak(self._tty.fileno())

        self._print_help()

    def _print_help(self) -> None:
        self.get_logger().info(
            "Keyboard teleop active (Ctrl+C to exit):\n"
            "  w/s: left stick up/down (joint2)\n"
            "  a/d: left stick left/right (joint1)\n"
            "  i/k: right stick up/down (joint3)\n"
            "  j/l: right stick left/right (joint4)\n"
            "  u/o: LB/RB buttons\n"
            "  space/m/n: A/B/Y buttons\n"
            "Hold keys to mimic stick deflection; OS key repeat keeps the axis nonzero."
        )

    def _read_keys(self) -> List[str]:
        keys = []
        try:
            while select.select([self._tty], [], [], 0)[0]:
                ch = self._tty.read(1)
                if ch:
                    keys.append(ch)
                else:
                    break
        except Exception:
            return keys
        return keys

    def _publish(self) -> None:
        axes = [0.0] * self.axis_count
        buttons = [0] * self.button_count

        for key in self._read_keys():
            if key == "\x03":  # Ctrl+C
                raise KeyboardInterrupt
            binding = KEY_BINDINGS.get(key)
            if binding is None:
                continue
            kind, index, value = binding
            if kind == "axis" and 0 <= index < self.axis_count:
                axes[index] = float(value)
            elif kind == "button" and 0 <= index < self.button_count:
                buttons[index] = int(value)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.publisher.publish(msg)

    def destroy_node(self) -> bool:
        if self._tty:
            termios.tcsetattr(self._tty, termios.TCSADRAIN, self._stdin_settings)
            if self._tty is not sys.stdin:
                self._tty.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KeyboardToJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
