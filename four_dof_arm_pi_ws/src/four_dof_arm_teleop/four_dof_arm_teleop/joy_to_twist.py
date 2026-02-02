from typing import List

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyToTwist(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_twist")
        # Axis order follows Xbox defaults unless overridden.
        self.declare_parameter("linear_axes", [1, 0, 4])  # x, y, z
        self.declare_parameter("angular_axes", [3, 2, 5])  # roll, pitch, yaw
        self.declare_parameter("linear_scale", [0.2, 0.2, 0.2])  # m/s per axis
        self.declare_parameter("angular_scale", [1.0, 1.0, 1.0])  # rad/s per axis
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("deadman_button", 4)  # LB to enable motion
        self.declare_parameter("publish_rate_hz", 50.0)

        self.linear_axes: List[int] = list(self.get_parameter("linear_axes").value)
        self.angular_axes: List[int] = list(self.get_parameter("angular_axes").value)
        self.linear_scale: List[float] = list(self.get_parameter("linear_scale").value)
        self.angular_scale: List[float] = list(self.get_parameter("angular_scale").value)
        self.deadzone: float = float(self.get_parameter("deadzone").value)
        self.deadman_button: int = int(self.get_parameter("deadman_button").value)
        self.publish_rate_hz: float = float(self.get_parameter("publish_rate_hz").value)

        self.latest_axes: List[float] = []
        self.latest_buttons: List[int] = []

        self.publisher = self.create_publisher(TwistStamped, "delta_twist_cmds", 10)
        self.create_subscription(Joy, "joy", self._joy_cb, 10)
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)

        self.get_logger().info(
            f"JoyToTwist: deadman button index {self.deadman_button}, deadzone {self.deadzone}, "
            f"linear axes {self.linear_axes} scales {self.linear_scale}, "
            f"angular axes {self.angular_axes} scales {self.angular_scale}"
        )

    def _joy_cb(self, msg: Joy) -> None:
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)

    def _extract(self, axes: List[int], scale: List[float]) -> List[float]:
        values = []
        for idx, axis_index in enumerate(axes):
            val = 0.0
            if 0 <= axis_index < len(self.latest_axes):
                val = float(self.latest_axes[axis_index])
            if abs(val) < self.deadzone:
                val = 0.0
            gain = scale[idx] if idx < len(scale) else 1.0
            values.append(val * gain)
        return values

    def _publish(self) -> None:
        if self.deadman_button >= 0:
            pressed = (
                self.deadman_button < len(self.latest_buttons)
                and self.latest_buttons[self.deadman_button] != 0
            )
            if not pressed:
                return

        lin = self._extract(self.linear_axes, self.linear_scale)
        ang = self._extract(self.angular_axes, self.angular_scale)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = lin[0] if len(lin) > 0 else 0.0
        msg.twist.linear.y = lin[1] if len(lin) > 1 else 0.0
        msg.twist.linear.z = lin[2] if len(lin) > 2 else 0.0
        msg.twist.angular.x = ang[0] if len(ang) > 0 else 0.0
        msg.twist.angular.y = ang[1] if len(ang) > 1 else 0.0
        msg.twist.angular.z = ang[2] if len(ang) > 2 else 0.0
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToTwist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
