from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint_names = LaunchConfiguration("joint_names")
    pwm_channels = LaunchConfiguration("pwm_channels")
    angle_min_rad = LaunchConfiguration("angle_min_rad")
    angle_max_rad = LaunchConfiguration("angle_max_rad")
    pulse_min_us = LaunchConfiguration("pulse_min_us")
    pulse_max_us = LaunchConfiguration("pulse_max_us")
    mavlink_connection = LaunchConfiguration("mavlink_connection")
    target_system = LaunchConfiguration("target_system")
    target_component = LaunchConfiguration("target_component")
    wait_heartbeat = LaunchConfiguration("wait_heartbeat")
    heartbeat_timeout_sec = LaunchConfiguration("heartbeat_timeout_sec")
    auto_target_from_heartbeat = LaunchConfiguration("auto_target_from_heartbeat")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joint_names", default_value="['joint1','joint2','joint3','joint4']"),
            DeclareLaunchArgument("pwm_channels", default_value="[9,10,11,12]", description="Navigator PWM channels"),
            DeclareLaunchArgument("angle_min_rad", default_value="[-2.6,-2.0,-2.6,-2.6]"),
            DeclareLaunchArgument("angle_max_rad", default_value="[2.6,2.6,2.6,2.6]"),
            DeclareLaunchArgument("pulse_min_us", default_value="700.0"),
            DeclareLaunchArgument("pulse_max_us", default_value="2300.0"),
            DeclareLaunchArgument(
                "mavlink_connection",
                default_value="udp:127.0.0.1:14550",
                description="MAVLink endpoint used to send DO_SET_SERVO",
            ),
            DeclareLaunchArgument("target_system", default_value="1"),
            DeclareLaunchArgument("target_component", default_value="1"),
            DeclareLaunchArgument("wait_heartbeat", default_value="true"),
            DeclareLaunchArgument("heartbeat_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("auto_target_from_heartbeat", default_value="true"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            Node(
                package="arm_pwm_bridge",
                executable="joint_traj_to_pwm",
                output="screen",
                parameters=[
                    {
                        "joint_names": joint_names,
                        "pwm_channels": pwm_channels,
                        "angle_min_rad": angle_min_rad,
                        "angle_max_rad": angle_max_rad,
                        "pulse_min_us": ParameterValue(pulse_min_us, value_type=float),
                        "pulse_max_us": ParameterValue(pulse_max_us, value_type=float),
                        "mavlink_connection": mavlink_connection,
                        "target_system": ParameterValue(target_system, value_type=int),
                        "target_component": ParameterValue(target_component, value_type=int),
                        "wait_heartbeat": ParameterValue(wait_heartbeat, value_type=bool),
                        "heartbeat_timeout_sec": ParameterValue(heartbeat_timeout_sec, value_type=float),
                        "auto_target_from_heartbeat": ParameterValue(auto_target_from_heartbeat, value_type=bool),
                        "publish_joint_states": ParameterValue(publish_joint_states, value_type=bool),
                    }
                ],
            ),
        ]
    )
