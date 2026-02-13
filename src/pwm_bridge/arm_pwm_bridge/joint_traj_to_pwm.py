import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    import pigpio  # type: ignore
except ImportError:  # pragma: no cover - runtime dependency, not present in CI by default
    pigpio = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """
    Minimal bridge: listens to JointTrajectory and drives Pi GPIO PWM via pigpio.
    Intended for the joy_to_trajectory teleop path (simple position updates).
    """

    def __init__(self) -> None:
        super().__init__("joint_traj_to_pwm")

        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("gpio_pins", [12, 13, 18, 19])  # BCM numbering
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("pigpio_host", "")
        self.declare_parameter("pigpio_port", 8888)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_rate_hz", 10.0)
        self.declare_parameter("stop_pwm_on_shutdown", True)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.gpio_pins: List[int] = [int(p) for p in list(self.get_parameter("gpio_pins").value)]
        self.pulse_min_us: float = float(self.get_parameter("pulse_min_us").value)
        self.pulse_max_us: float = float(self.get_parameter("pulse_max_us").value)
        self.angle_min_rad: List[float] = list(self.get_parameter("angle_min_rad").value)
        self.angle_max_rad: List[float] = list(self.get_parameter("angle_max_rad").value)
        self.initial_positions_rad: List[float] = list(self.get_parameter("initial_positions_rad").value)
        self.pigpio_host: str = str(self.get_parameter("pigpio_host").value).strip()
        self.pigpio_port: int = int(self.get_parameter("pigpio_port").value)
        self.publish_joint_states: bool = bool(self.get_parameter("publish_joint_states").value)
        self.joint_state_rate_hz: float = float(self.get_parameter("joint_state_rate_hz").value)
        self.stop_pwm_on_shutdown: bool = bool(self.get_parameter("stop_pwm_on_shutdown").value)

        if len(self.joint_names) != len(self.gpio_pins):
            self.get_logger().warn(
                f"joint_names ({len(self.joint_names)}) and gpio_pins ({len(self.gpio_pins)}) length mismatch; "
                "extra joints or pins will be ignored."
            )

        self._pi: Optional["pigpio.pi"] = None
        self._pigpio_ready = False
        self._warned_disabled = False
        self._last_positions: Dict[str, float] = {
            name: self._value_for_idx(self.initial_positions_rad, idx, 0.0)
            for idx, name in enumerate(self.joint_names)
        }

        self._connect_pigpio()

        self.create_subscription(JointTrajectory, self.trajectory_topic, self._traj_cb, 10)
        self.get_logger().info(
            f"Listening on {self.trajectory_topic} for JointTrajectory commands -> PWM."
        )

        if self.publish_joint_states:
            self._js_pub = self.create_publisher(JointState, "joint_states", 10)
            self.create_timer(1.0 / self.joint_state_rate_hz, self._publish_joint_states)
        else:
            self._js_pub = None

        # Apply initial positions if specified.
        if self._pigpio_ready:
            self._write_positions(self._last_positions, log=False)

    def _value_for_idx(self, values: List[float], idx: int, default: float) -> float:
        return float(values[idx]) if idx < len(values) else default

    def _connect_pigpio(self) -> None:
        if pigpio is None:
            self.get_logger().error(
                "pigpio is not installed. Install python3-pigpio and start pigpiod to enable PWM output."
            )
            return

        host = self.pigpio_host if self.pigpio_host else None
        pi = pigpio.pi() if host is None else pigpio.pi(host, self.pigpio_port)
        if not pi.connected:
            self.get_logger().error(
                f"Failed to connect to pigpio daemon (host={host or 'localhost'}, port={self.pigpio_port}). "
                "Start it with: sudo pigpiod"
            )
            return

        for pin in self.gpio_pins:
            pi.set_mode(pin, pigpio.OUTPUT)
            # pigpio servo helper sends 50 Hz pulses automatically.
        self._pi = pi
        self._pigpio_ready = True
        self.get_logger().info(
            f"pigpio connected ({'localhost' if host is None else host}:{self.pigpio_port}); "
            f"driving pins {self.gpio_pins}."
        )

    def _traj_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        if not self._pigpio_ready:
            if not self._warned_disabled:
                self.get_logger().warn("PWM disabled (pigpio not ready); dropping JointTrajectory messages.")
                self._warned_disabled = True
            return

        point = msg.points[-1]
        name_to_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        updates: Dict[str, float] = {}

        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.gpio_pins):
                continue
            pin = self.gpio_pins[idx]
            js_idx = name_to_index.get(joint)
            if js_idx is None:
                continue
            if js_idx >= len(point.positions):
                continue

            angle = float(point.positions[js_idx])
            clamped = self._clamp_angle(idx, angle)
            pulse_us = self._angle_to_pulse(idx, clamped)
            self._pi.set_servo_pulsewidth(pin, pulse_us)
            updates[joint] = clamped

        if updates:
            self._last_positions.update(updates)

    def _clamp_angle(self, idx: int, angle: float) -> float:
        lower = self._value_for_idx(self.angle_min_rad, idx, -math.pi)
        upper = self._value_for_idx(self.angle_max_rad, idx, math.pi)
        if upper <= lower:
            return angle
        return clamp(angle, lower, upper)

    def _angle_to_pulse(self, idx: int, angle: float) -> float:
        lower = self._value_for_idx(self.angle_min_rad, idx, -math.pi)
        upper = self._value_for_idx(self.angle_max_rad, idx, math.pi)
        pulse_min = self.pulse_min_us
        pulse_max = self.pulse_max_us
        if upper <= lower or pulse_max <= pulse_min:
            return pulse_min
        ratio = (angle - lower) / (upper - lower)
        return clamp(pulse_min + ratio * (pulse_max - pulse_min), pulse_min, pulse_max)

    def _publish_joint_states(self) -> None:
        if not self._js_pub:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = [self._last_positions.get(name, 0.0) for name in self.joint_names]
        self._js_pub.publish(msg)

    def _write_positions(self, positions: Dict[str, float], log: bool = True) -> None:
        if not self._pigpio_ready:
            return
        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.gpio_pins):
                continue
            pin = self.gpio_pins[idx]
            angle = positions.get(joint, 0.0)
            pulse_us = self._angle_to_pulse(idx, self._clamp_angle(idx, angle))
            self._pi.set_servo_pulsewidth(pin, pulse_us)
        if log:
            self.get_logger().info(f"Applied initial positions (us in [{self.pulse_min_us}, {self.pulse_max_us}]).")

    def destroy_node(self) -> bool:
        if self._pi and self._pigpio_ready and self.stop_pwm_on_shutdown:
            for pin in self.gpio_pins:
                self._pi.set_servo_pulsewidth(pin, 0)  # 0 stops pulses
            self._pi.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTrajectoryToPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
