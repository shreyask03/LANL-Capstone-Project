from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JoyToTrajectory(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_trajectory")

        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("axis_map", [0, 1, 4, 3])
        self.declare_parameter("velocity_scale", 1.2)  # rad/s at full deflection
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("axis_slew_rate", 5.0)  # max axis change per second (for smoothing)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("time_from_start", 0.0)  # <=0 means use 1/publish_rate_hz
        self.declare_parameter("lower_limits", [-3.14, -3.14, -3.14, -3.14])
        self.declare_parameter("upper_limits", [3.14, 3.14, 3.14, 3.14])
        self.declare_parameter("reset_button_index", 1)  # default: B button
        self.declare_parameter("initial_positions", [0.0, 0.0, 0.0, 0.0])

        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.axis_map: List[int] = list(self.get_parameter("axis_map").value)
        self.velocity_scale: float = float(self.get_parameter("velocity_scale").value)
        self.deadzone: float = float(self.get_parameter("deadzone").value)
        self.axis_slew_rate: float = float(self.get_parameter("axis_slew_rate").value)
        self.publish_rate_hz: float = float(self.get_parameter("publish_rate_hz").value)
        configured_time_from_start: float = float(self.get_parameter("time_from_start").value)
        self.lower_limits: List[float] = list(self.get_parameter("lower_limits").value)
        self.upper_limits: List[float] = list(self.get_parameter("upper_limits").value)
        self.reset_button_index: int = int(self.get_parameter("reset_button_index").value)
        self.initial_positions: List[float] = list(self.get_parameter("initial_positions").value)

        self.dt = 1.0 / self.publish_rate_hz
        # Default to dt when time_from_start <= 0 for snappier response; otherwise respect the parameter.
        self.time_from_start = self.dt if configured_time_from_start <= 0 else configured_time_from_start

        self.current_positions: List[float] = list(self.initial_positions)
        self.latest_axes: List[float] = [0.0] * max(len(self.axis_map), 8)
        self.latest_buttons: List[int] = []
        self.filtered_axes: List[float] = [0.0] * max(len(self.axis_map), 8)

        if len(self.axis_map) != len(self.joint_names):
            self.get_logger().warn(
                f"axis_map length ({len(self.axis_map)}) != joint_names length ({len(self.joint_names)}); "
                "unmapped joints will not move."
            )

        self.publisher = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.create_subscription(Joy, "joy", self._joy_cb, 10)
        self.timer = self.create_timer(self.dt, self._publish)

        self.get_logger().info(
            f"Mapping axes {self.axis_map} to joints {self.joint_names}. "
            f"Velocity scale {self.velocity_scale} rad/s, deadzone {self.deadzone}, "
            f"axis_slew_rate {self.axis_slew_rate}/s, time_from_start {self.time_from_start}s, "
            f"publish_rate {self.publish_rate_hz} Hz."
        )

    def _joy_cb(self, msg: Joy) -> None:
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)
        if len(self.filtered_axes) < len(msg.axes):
            self.filtered_axes.extend([0.0] * (len(msg.axes) - len(self.filtered_axes)))

    def _publish(self) -> None:
        dt = self.dt

        if self.reset_button_index < len(self.latest_buttons) and self.latest_buttons[self.reset_button_index]:
            self.current_positions = list(self.initial_positions)

        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.axis_map):
                continue
            axis_index = self.axis_map[idx]
            raw_axis = 0.0
            if 0 <= axis_index < len(self.latest_axes):
                raw_axis = float(self.latest_axes[axis_index])
            if abs(raw_axis) < self.deadzone:
                raw_axis = 0.0

            if axis_index >= len(self.filtered_axes):
                self.filtered_axes.extend([0.0] * (axis_index - len(self.filtered_axes) + 1))

            max_step = self.axis_slew_rate * dt
            step = clamp(raw_axis - self.filtered_axes[axis_index], -max_step, max_step)
            self.filtered_axes[axis_index] += step
            axis_value = self.filtered_axes[axis_index]

            delta = axis_value * self.velocity_scale * dt
            lower = self.lower_limits[idx] if idx < len(self.lower_limits) else -3.14
            upper = self.upper_limits[idx] if idx < len(self.upper_limits) else 3.14
            self.current_positions[idx] = clamp(self.current_positions[idx] + delta, lower, upper)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(self.current_positions)
        point.time_from_start = Duration(
            sec=int(self.time_from_start),
            nanosec=int((self.time_from_start - int(self.time_from_start)) * 1e9),
        )

        traj.points.append(point)
        self.publisher.publish(traj)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
