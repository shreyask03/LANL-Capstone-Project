import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from pymavlink import mavutil  # type: ignore
except ImportError:  # pragma: no cover - runtime dependency, not present in CI by default
    mavutil = None


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class JointTrajectoryToPWM(Node):
    """
    Minimal bridge: listens to JointTrajectory and drives Navigator PWM channels
    via MAVLink DO_SET_SERVO commands.
    """

    def __init__(self) -> None:
        super().__init__("joint_traj_to_pwm")

        self.declare_parameter("trajectory_topic", "arm_controller/joint_trajectory")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("pwm_channels", [9, 10, 11, 12])
        self.declare_parameter("pulse_min_us", 700.0)
        self.declare_parameter("pulse_max_us", 2300.0)
        self.declare_parameter("angle_min_rad", [-2.6, -2.0, -2.6, -2.6])
        self.declare_parameter("angle_max_rad", [2.6, 2.6, 2.6, 2.6])
        self.declare_parameter("initial_positions_rad", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("mavlink_connection", "udp:127.0.0.1:14550")
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("wait_heartbeat", True)
        self.declare_parameter("heartbeat_timeout_sec", 5.0)
        self.declare_parameter("auto_target_from_heartbeat", True)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter("joint_state_rate_hz", 10.0)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        self.pwm_channels: List[int] = [int(ch) for ch in list(self.get_parameter("pwm_channels").value)]
        self.pulse_min_us: float = float(self.get_parameter("pulse_min_us").value)
        self.pulse_max_us: float = float(self.get_parameter("pulse_max_us").value)
        self.angle_min_rad: List[float] = list(self.get_parameter("angle_min_rad").value)
        self.angle_max_rad: List[float] = list(self.get_parameter("angle_max_rad").value)
        self.initial_positions_rad: List[float] = list(self.get_parameter("initial_positions_rad").value)
        self.mavlink_connection: str = str(self.get_parameter("mavlink_connection").value).strip()
        self.target_system: int = int(self.get_parameter("target_system").value)
        self.target_component: int = int(self.get_parameter("target_component").value)
        self.wait_heartbeat: bool = bool(self.get_parameter("wait_heartbeat").value)
        self.heartbeat_timeout_sec: float = float(self.get_parameter("heartbeat_timeout_sec").value)
        self.auto_target_from_heartbeat: bool = bool(self.get_parameter("auto_target_from_heartbeat").value)
        self.publish_joint_states: bool = bool(self.get_parameter("publish_joint_states").value)
        self.joint_state_rate_hz: float = float(self.get_parameter("joint_state_rate_hz").value)

        if len(self.joint_names) != len(self.pwm_channels):
            self.get_logger().warn(
                f"joint_names ({len(self.joint_names)}) and pwm_channels ({len(self.pwm_channels)}) length mismatch; "
                "extra joints or channels will be ignored."
            )

        self._mav = None
        self._mavlink_ready = False
        self._warned_disabled = False
        self._last_positions: Dict[str, float] = {
            name: self._value_for_idx(self.initial_positions_rad, idx, 0.0)
            for idx, name in enumerate(self.joint_names)
        }

        self._connect_mavlink()

        self.create_subscription(JointTrajectory, self.trajectory_topic, self._traj_cb, 10)
        self.get_logger().info(
            f"Listening on {self.trajectory_topic} for JointTrajectory commands -> Navigator PWM."
        )

        if self.publish_joint_states:
            self._js_pub = self.create_publisher(JointState, "joint_states", 10)
            self.create_timer(1.0 / self.joint_state_rate_hz, self._publish_joint_states)
        else:
            self._js_pub = None

        # Apply initial positions if specified.
        if self._mavlink_ready:
            self._write_positions(self._last_positions, log=False)

    def _value_for_idx(self, values: List[float], idx: int, default: float) -> float:
        return float(values[idx]) if idx < len(values) else default

    def _connect_mavlink(self) -> None:
        if mavutil is None:
            self.get_logger().error(
                "pymavlink is not installed. Install python3-pymavlink to enable Navigator PWM output."
            )
            return

        if not self.mavlink_connection:
            self.get_logger().error(
                "mavlink_connection is empty. Set it to a valid endpoint, e.g. udp:127.0.0.1:14550."
            )
            return

        try:
            self._mav = mavutil.mavlink_connection(self.mavlink_connection)
        except Exception as exc:
            self.get_logger().error(f"Failed opening MAVLink connection '{self.mavlink_connection}': {exc}")
            return

        if self.wait_heartbeat:
            try:
                heartbeat = self._mav.wait_heartbeat(timeout=self.heartbeat_timeout_sec)
            except Exception as exc:
                self.get_logger().error(
                    f"No MAVLink heartbeat on '{self.mavlink_connection}' within "
                    f"{self.heartbeat_timeout_sec}s: {exc}"
                )
                return
            if heartbeat is not None and self.auto_target_from_heartbeat:
                self.target_system = int(heartbeat.get_srcSystem())
                self.target_component = int(heartbeat.get_srcComponent())

        for channel in self.pwm_channels:
            if channel <= 0:
                self.get_logger().warn(
                    f"Invalid pwm channel {channel}; channels should be positive (Navigator: 1-16)."
                )

        self._mavlink_ready = True
        self.get_logger().info(
            f"MAVLink connected on {self.mavlink_connection}; "
            f"target={self.target_system}:{self.target_component}; "
            f"driving channels {self.pwm_channels}."
        )

    def _traj_cb(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        if not self._mavlink_ready:
            if not self._warned_disabled:
                self.get_logger().warn("PWM disabled (MAVLink not ready); dropping JointTrajectory messages.")
                self._warned_disabled = True
            return

        point = msg.points[-1]
        name_to_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        updates: Dict[str, float] = {}

        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.pwm_channels):
                continue
            channel = self.pwm_channels[idx]
            js_idx = name_to_index.get(joint)
            if js_idx is None:
                continue
            if js_idx >= len(point.positions):
                continue

            angle = float(point.positions[js_idx])
            clamped = self._clamp_angle(idx, angle)
            pulse_us = self._angle_to_pulse(idx, clamped)
            self._set_servo(channel, pulse_us)
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

    def _set_servo(self, channel: int, pulse_us: float) -> None:
        if not self._mavlink_ready or self._mav is None:
            return
        if channel <= 0:
            return
        try:
            self._mav.mav.command_long_send(
                self.target_system,
                self.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                float(channel),
                float(pulse_us),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        except Exception as exc:
            self.get_logger().error(f"Failed sending DO_SET_SERVO for channel {channel}: {exc}")

    def _write_positions(self, positions: Dict[str, float], log: bool = True) -> None:
        if not self._mavlink_ready:
            return
        for idx, joint in enumerate(self.joint_names):
            if idx >= len(self.pwm_channels):
                continue
            channel = self.pwm_channels[idx]
            angle = positions.get(joint, 0.0)
            pulse_us = self._angle_to_pulse(idx, self._clamp_angle(idx, angle))
            self._set_servo(channel, pulse_us)
        if log:
            self.get_logger().info(f"Applied initial positions (us in [{self.pulse_min_us}, {self.pulse_max_us}]).")

    def destroy_node(self) -> bool:
        if self._mav:
            try:
                self._mav.close()
            except Exception:
                pass
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
