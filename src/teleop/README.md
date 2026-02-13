# arm_teleop

Keyboard-first teleoperation that mimics Xbox-style joystick inputs for the 4-DOF arm. `keyboard_to_joy` publishes `sensor_msgs/Joy`, and `joy_to_trajectory` maps stick deflections to incremental joint-space trajectories for the `arm_controller`.

## Key mappings
- `w/s`: left stick up/down (joint2)
- `a/d`: left stick left/right (joint1)
- `i/k`: right stick up/down (joint3)
- `j/l`: right stick left/right (joint4)
- `space`: A button, `m`: B button, `n`: Y button, `u/o`: LB/RB
- Hold a key to keep a deflection (OS key repeat keeps the axis nonzero).

## Typical run
1) Build and source the workspace: `colcon build --symlink-install && source install/setup.bash`.
2) Bring up ros2_control (controllers + robot_state_publisher):  
   `ros2 launch arm_bringup ros2_control.launch.py`
3) Start the PWM bridge if you're driving real servos:  
   `ros2 launch arm_pwm_bridge pwm_bridge.launch.py`
4) Start the keyboard teleop stack (Joy publisher + joint trajectory mapper):  
   `ros2 launch arm_launch keyboard_teleop.launch.py velocity_scale:=1.2`
   - Adjust stick speed with `velocity_scale:=<rad_per_sec>` and deadzone with `deadzone:=<value>`.
   - B button (`m`) resets joint targets to zero.

## Swap to a real gamepad later
- Replace `keyboard_to_joy` with `ros2 run joy joy_node` and keep `joy_to_trajectory` running; it listens on `/joy`.
- Axis order defaults to `axis_map=[0,1,4,3]` for (joint1..4). Change via parameters if your gamepad order differs.
