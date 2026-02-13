# arm_pwm_bridge

`JointTrajectory` → PWM bridge for the 4-DOF arm on a Raspberry Pi. It listens on `arm_controller/joint_trajectory` (what `joy_to_trajectory` publishes) and outputs 50 Hz servo pulses via `pigpio`.

## Prereqs
- Install pigpio and start its daemon:
  ```bash
  sudo apt install -y python3-pigpio pigpio
  sudo systemctl start pigpiod    # or: sudo pigpiod
  ```
- Wire your servos to the Pi (BCM pins, default `[12,13,18,19]`) and power them from an adequate 5V rail.

## Build
```bash
cd /path/to/arm_control
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select arm_pwm_bridge arm_teleop
source install/setup.bash
```

## Run on hardware (no Gazebo)
Terminal 1: ros2_control bringup (controllers + state publisher):
```bash
ros2 launch arm_bringup ros2_control.launch.py
```

Terminal 2: keyboard/gamepad teleop to publish joint targets:
```bash
ros2 launch arm_launch keyboard_teleop.launch.py use_sim_time:=false
```

Terminal 3: PWM bridge:
```bash
ros2 launch arm_pwm_bridge pwm_bridge.launch.py \
  gpio_pins:=[12,13,18,19] \
  angle_min_rad:=[-2.6,-2.0,-2.6,-2.6] \
  angle_max_rad:=[2.6,2.6,2.6,2.6] \
  pulse_min_us:=700.0 pulse_max_us:=2300.0
```

## Parameters (node or launch override)
- `trajectory_topic` (string): topic to listen for JointTrajectory commands (default `arm_controller/joint_trajectory`).
- `joint_names` (string list): joints to map, order aligned with pins.
- `gpio_pins` (int list): BCM GPIO pins for each joint.
- `angle_min_rad` / `angle_max_rad` (float list): clamp ranges per joint.
- `pulse_min_us` / `pulse_max_us` (float): microsecond range sent to servos.
- `initial_positions_rad` (float list): optional starting setpoint.
- `pigpio_host` / `pigpio_port`: pigpio daemon location (blank host = localhost).
- `publish_joint_states` (bool): republish the last commanded angles for RViz.
- `stop_pwm_on_shutdown` (bool): send 0 pulsewidth when shutting down.

Notes:
- This bridge suits the simple `joy_to_trajectory` path. It does not implement FollowJointTrajectory actions (used by MoveIt); add a full ros2_control hardware interface if you need that.
- `pigpio` keeps generating pulses after you set a width; `stop_pwm_on_shutdown` sends 0 to release the servos. Adjust ranges to your servo geometry before use. 
