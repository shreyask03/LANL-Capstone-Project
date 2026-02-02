# four_dof_arm_teleop

Keyboard-first teleoperation that mimics Xbox-style joystick inputs for the 4-DOF arm. `keyboard_to_joy` publishes `sensor_msgs/Joy`, and `joy_to_trajectory` maps stick deflections to incremental joint trajectories for the `arm_controller`.

## Key mappings
- `w/s`: left stick up/down (joint2)
- `a/d`: left stick left/right (joint1)
- `i/k`: right stick up/down (joint3)
- `j/l`: right stick left/right (joint4)
- `space`: A button, `m`: B button, `n`: Y button, `u/o`: LB/RB
- Hold a key to keep a deflection (OS key repeat keeps the axis nonzero).

## Run with Gazebo
1) Build and source:
   ```bash
   colcon build --packages-select four_dof_arm_description four_dof_arm_bringup four_dof_arm_moveit_config four_dof_arm_teleop
   source install/setup.bash
   ```
2) Start the sim (Gazebo + controllers):
   ```bash
   ros2 launch four_dof_arm_bringup gazebo.launch.py
   ```
3) In a new terminal (sourced), start keyboard teleop:
   ```bash
   ros2 launch four_dof_arm_teleop keyboard_teleop.launch.py
   ```
   - Adjust stick speed with `velocity_scale:=<rad_per_sec>` and deadzone with `deadzone:=<value>`.
   - B button (`m`) resets joint targets to zero.

## Swap to a real gamepad later
- Replace `keyboard_to_joy` with `ros2 run joy joy_node` and keep `joy_to_trajectory` running; it listens on `/joy`.
- Axis order defaults to `axis_map=[0,1,4,3]` for (joint1..4). Change via parameters if your gamepad order differs.

## Cartesian-ish teleop (separate terminal)
Uses MoveIt IK to step the end-effector in Cartesian space (no Servo).
1) Start the sim (Terminal 1): `ros2 launch four_dof_arm_bringup gazebo.launch.py`
2) Control terminal (Terminal 2): `ros2 launch four_dof_arm_teleop cartesian_teleop.launch.py`
   - Left stick: X/Y translation; right stick up/down: Z; right stick left/right: yaw.
   - Deadman: LB (`u`) must be held to move.
   - Adjust speed: `linear_scale` (m/s), `angular_scale` (rad/s) params on `joy_to_pose_ik`.
3) To use a real gamepad, replace `keyboard_to_joy` with `ros2 run joy joy_node`; `joy_to_pose_ik` still listens on `/joy`.
