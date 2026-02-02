# four_dof_arm_description

Placeholder URDF/XACRO for a 4-DOF arm that can be driven in Gazebo and MoveIt2.

## Swap in your CAD (SLDPRT) meshes
1. Export each SLDPRT as `STL` or `DAE` (one file per link).
2. Replace the simple primitives in `urdf/four_dof_arm.urdf.xacro` with mesh references for your CAD if you want visuals to match your geometry.
3. The current visuals use large boxes/cylinders so they are easy to see in Gazebo. When swapping to meshes, set an appropriate `<mesh scale>` to match your export units (e.g., `0.001` for millimeter CAD).

## Adjust kinematics/dynamics
- Change the `origin` and `axis` attributes on `joint1`..`joint4` to modify joint relations or axes.
- Update the `limit` tags for travel/velocity/effort, and the mass/inertia values in the `default_inertial` macro calls for realistic dynamics.
- Link lengths are defined by `link1_length`..`link4_length` properties near the top of the Xacro.

## Simulation + control wiring
- `urdf/ros2_control.xacro` defines ros2_control interfaces for Gazebo (`gazebo_ros2_control/GazeboSystem`).
- Controller parameters live in `four_dof_arm_bringup/config/ros2_controllers.yaml` and are referenced through the `controllers_file` Xacro argument.
