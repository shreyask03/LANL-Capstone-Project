# four_dof_arm_pi_ws

Isolated ROS 2 workspace containing only the four_dof_arm packages (description, bringup, MoveIt config, teleop) with all ros2_control example packages excluded.

## Using on the Raspberry Pi
- Copy `four_dof_arm_pi_src.tar.gz` to the Pi and unpack into your ROS 2 workspace: `tar -xzf four_dof_arm_pi_src.tar.gz -C ~/ros2_ws/src` (or drop the `four_dof_arm_pi_ws/src` folder directly into `src`).
- Install dependencies from the workspace root: `rosdep install --from-paths src -y --ignore-src`.
- Build: `colcon build --symlink-install`.
- Source the overlay and launch as needed, for example:
  - `ros2 launch four_dof_arm_bringup gazebo.launch.py` for Gazebo simulation.
  - `ros2 launch four_dof_arm_moveit_config move_group.launch.py` for MoveIt.
  - `ros2 launch four_dof_arm_teleop keyboard_servo_teleop.launch.py` for keyboard teleop.

