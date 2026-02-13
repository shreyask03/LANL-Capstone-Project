# arm_description

Placeholder URDF/XACRO for a 4-DOF arm that can be driven with ros2_control and MoveIt2 (no Gazebo pieces).

## Swap in your CAD (SLDPRT) meshes
1. Export each SLDPRT as `STL` or `DAE` (one file per link).
2. Replace the simple primitives in `urdf/four_dof_arm.urdf.xacro` with mesh references for your CAD if you want visuals to match your geometry.
3. Set an appropriate `<mesh scale>` to match your export units (e.g., `0.001` for millimeter CAD).

## Adjust kinematics/dynamics
- Change the `origin` and `axis` attributes on `joint1`..`joint4` to modify joint relations or axes.
- Update the `limit` tags for travel/velocity/effort, and the mass/inertia values in the `default_inertial` macro calls for realistic dynamics.
- Link lengths are defined by `link1_length`..`link4_length` properties near the top of the Xacro.

## Simulation + control wiring
- `urdf/ros2_control.xacro` defines ros2_control interfaces using a simple GenericSystem plugin so the controllers can run without Gazebo.
- Controller parameters live in `arm_config/config/ros2_controllers.yaml` and are referenced through the `controllers_file` Xacro argument.
