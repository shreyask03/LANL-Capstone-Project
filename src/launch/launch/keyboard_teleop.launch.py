from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    velocity_scale = LaunchConfiguration("velocity_scale")
    deadzone = LaunchConfiguration("deadzone")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    time_from_start = LaunchConfiguration("time_from_start")
    axis_slew_rate = LaunchConfiguration("axis_slew_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("velocity_scale", default_value="1.2", description="rad/s at full stick"),
            DeclareLaunchArgument("deadzone", default_value="0.05"),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0", description="Joint trajectory publish rate"),
            DeclareLaunchArgument(
                "time_from_start",
                default_value="0.0",
                description="Trajectory point horizon (s). 0 -> use 1/publish_rate_hz for snappier stops.",
            ),
            DeclareLaunchArgument(
                "axis_slew_rate",
                default_value="5.0",
                description="Max stick change per second (smoothing). Higher = snappier, lower = smoother.",
            ),
            Node(
                package="arm_teleop",
                executable="keyboard_to_joy",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="arm_teleop",
                executable="joy_to_trajectory",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "velocity_scale": ParameterValue(velocity_scale, value_type=float),
                        "deadzone": ParameterValue(deadzone, value_type=float),
                        "publish_rate_hz": ParameterValue(publish_rate_hz, value_type=float),
                        "time_from_start": ParameterValue(time_from_start, value_type=float),
                        "axis_slew_rate": ParameterValue(axis_slew_rate, value_type=float),
                    }
                ],
            ),
        ]
    )
