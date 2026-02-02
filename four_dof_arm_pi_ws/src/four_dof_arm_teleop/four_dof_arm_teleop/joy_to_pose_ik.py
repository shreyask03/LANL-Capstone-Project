import math
from typing import List, Optional

import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

LINK1 = 1.27
LINK2 = 1.27
LINK3 = 1.27


def rot_z(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def rot_x(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])


def trans(x: float, y: float, z: float) -> np.ndarray:
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


class JoyToPoseIK(Node):
    def __init__(self) -> None:
        super().__init__("joy_to_pose_ik")
        self.declare_parameter("linear_scale", 1.5)  # aggressive Cartesian delta
        self.declare_parameter("angular_scale", 2.5)  # aggressive yaw delta
        self.declare_parameter("deadzone", 0.05)
        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("max_step_rad", 0.8)
        self.declare_parameter("damping", 0.02)
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("controller_name", "arm_controller")

        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.angular_scale = float(self.get_parameter("angular_scale").value)
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.deadman_button = int(self.get_parameter("deadman_button").value)
        self.rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.max_step = float(self.get_parameter("max_step_rad").value)
        self.damping = float(self.get_parameter("damping").value)
        self.joint_names: List[str] = list(self.get_parameter("joint_names").value)
        controller = str(self.get_parameter("controller_name").value)

        self.latest_axes: List[float] = []
        self.latest_buttons: List[int] = []
        self.latest_joint_state: Optional[JointState] = None

        self.create_subscription(Joy, "joy", self._joy_cb, 10)
        self.create_subscription(JointState, "joint_states", self._joint_state_cb, 10)
        self.create_timer(1.0 / self.rate_hz, self._tick)

        self.action_client = ActionClient(self, FollowJointTrajectory, f"{controller}/follow_joint_trajectory")

        self.get_logger().info(
            f"JoyToPoseIK running (no MoveIt). Deadman {self.deadman_button}; "
            f"linear_scale {self.linear_scale} m/s, angular_scale {self.angular_scale} rad/s."
        )

    def _joy_cb(self, msg: Joy) -> None:
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _deadman(self) -> bool:
        return True

    def _tick(self) -> None:
        if not self.latest_axes or self.latest_joint_state is None:
            return

        q = self._extract_joints(self.latest_joint_state)
        if q is None:
            return

        dt = 1.0 / self.rate_hz
        desired_delta = np.array(
            [
                self._axis(0) * self.linear_scale * dt,  # dx
                self._axis(1) * self.linear_scale * dt,  # dy
                self._axis(4) * self.linear_scale * dt,  # dz
                self._axis(3) * self.angular_scale * dt,  # dyaw
            ]
        )
        if np.linalg.norm(desired_delta) < 1e-6:
            return

        pos, yaw = self._fk(q)
        J = self._numeric_jacobian(q, pos, yaw)
        try:
            JT = J.T
            lam2 = self.damping * self.damping
            dq = np.linalg.solve(JT @ J + lam2 * np.eye(J.shape[1]), JT @ desired_delta)
        except np.linalg.LinAlgError:
            try:
                dq = np.linalg.pinv(J).dot(desired_delta)
            except np.linalg.LinAlgError:
                # Fallback: map axes directly to small joint deltas
                dq = np.array(
                    [
                        self._axis(0) * self.linear_scale * dt,
                        self._axis(1) * self.angular_scale * dt,
                        self._axis(4) * self.linear_scale * dt,
                        self._axis(3) * self.angular_scale * dt,
                    ]
                )

        dq_clamped = np.clip(dq, -self.max_step, self.max_step)
        new_q = q + dq_clamped
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(new_q)
        point.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points.append(point)

        if not self.action_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("Controller action server not available", throttle_duration_sec=2.0)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.action_client.send_goal_async(goal)

    def _extract_joints(self, js: JointState) -> Optional[np.ndarray]:
        vals = []
        for name in self.joint_names:
            if name not in js.name:
                return None
            vals.append(js.position[js.name.index(name)])
        return np.array(vals, dtype=float)

    def _fk(self, q: np.ndarray) -> (np.ndarray, float):
        T = (
            trans(0, 0, 0.1)
            @ rot_z(q[0])
            @ trans(0, 0, -LINK1)
            @ rot_x(q[1])
            @ trans(LINK2, 0, 0)
            @ rot_z(q[2])
            @ trans(0, 0, -LINK3)
            @ rot_x(q[3])
        )
        pos = T[:3, 3]
        yaw = math.atan2(T[1, 0], T[0, 0])
        return pos, yaw

    def _numeric_jacobian(self, q: np.ndarray, base_pos: np.ndarray, base_yaw: float) -> np.ndarray:
        eps = 1e-4
        J = np.zeros((4, len(q)))
        for i in range(len(q)):
            dq = np.zeros_like(q)
            dq[i] = eps
            pos_eps, yaw_eps = self._fk(q + dq)
            delta = np.hstack((pos_eps - base_pos, [self._wrap_angle(yaw_eps - base_yaw)]))
            J[:, i] = delta / eps
        return J

    def _axis(self, index: int) -> float:
        if 0 <= index < len(self.latest_axes):
            val = float(self.latest_axes[index])
            return 0.0 if abs(val) < self.deadzone else val
        return 0.0

    def _wrap_angle(self, ang: float) -> float:
        return (ang + math.pi) % (2 * math.pi) - math.pi


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyToPoseIK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
