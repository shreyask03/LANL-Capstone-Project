# arm_pwm_bridge

`JointTrajectory` -> PWM bridge for the 4-DOF arm using MAVLink `DO_SET_SERVO`.
It listens on `arm_controller/joint_trajectory` (what `joy_to_trajectory` publishes)
and commands Navigator PWM outputs (for example channels `9-16`).

## Prereqs
- Install pymavlink:
  ```bash
  sudo apt install -y python3-pymavlink
  ```
- Configure a MAVLink endpoint in BlueOS for this app/container.
- Ensure output channels are configured in ArduPilot (`SERVOx_FUNCTION`) for direct servo control.

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
  pwm_channels:=[9,10,11,12] \
  mavlink_connection:=udp:127.0.0.1:14550 \
  angle_min_rad:=[-2.6,-2.0,-2.6,-2.6] \
  angle_max_rad:=[2.6,2.6,2.6,2.6] \
  pulse_min_us:=700.0 pulse_max_us:=2300.0
```

## Parameters (node or launch override)
- `trajectory_topic` (string): topic to listen for JointTrajectory commands (default `arm_controller/joint_trajectory`).
- `joint_names` (string list): joints to map, order aligned with channels.
- `pwm_channels` (int list): output channels to drive (Navigator supports `1..16`).
- `angle_min_rad` / `angle_max_rad` (float list): clamp ranges per joint.
- `pulse_min_us` / `pulse_max_us` (float): microsecond range sent to servos.
- `initial_positions_rad` (float list): optional starting setpoint.
- `mavlink_connection` (string): pymavlink endpoint (e.g. `udp:127.0.0.1:14550`).
- `target_system` / `target_component` (int): MAVLink target IDs.
- `wait_heartbeat` (bool): wait for heartbeat before sending commands.
- `heartbeat_timeout_sec` (float): heartbeat wait timeout.
- `auto_target_from_heartbeat` (bool): override target IDs from received heartbeat.
- `publish_joint_states` (bool): republish the last commanded angles for RViz.

Notes:
- This bridge suits the simple `joy_to_trajectory` path. It does not implement FollowJointTrajectory actions (used by MoveIt); add a full ros2_control hardware interface if you need that.
- Each command is sent as MAVLink `MAV_CMD_DO_SET_SERVO` with `param1=channel`, `param2=pulse_us`. Adjust ranges to your servo geometry before use.
