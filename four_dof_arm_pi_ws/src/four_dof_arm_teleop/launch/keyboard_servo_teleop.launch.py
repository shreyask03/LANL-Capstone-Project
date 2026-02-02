from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    deadzone = LaunchConfiguration("deadzone")
    linear_scale = LaunchConfiguration("linear_scale")
    angular_scale = LaunchConfiguration("angular_scale")

    linear_scale_val = ParameterValue(linear_scale, value_type=float)
    angular_scale_val = ParameterValue(angular_scale, value_type=float)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("deadzone", default_value="0.05"),
            DeclareLaunchArgument("linear_scale", default_value="0.2", description="servo linear m/s at full stick"),
            DeclareLaunchArgument("angular_scale", default_value="1.0", description="servo angular rad/s at full stick"),
            Node(
                package="four_dof_arm_teleop",
                executable="keyboard_to_joy",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="four_dof_arm_teleop",
                executable="joy_to_twist",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "deadzone": ParameterValue(deadzone, value_type=float),
                        "linear_scale": [linear_scale_val, linear_scale_val, linear_scale_val],
                        "angular_scale": [angular_scale_val, angular_scale_val, angular_scale_val],
                    }
                ],
            ),
        ]
    )
