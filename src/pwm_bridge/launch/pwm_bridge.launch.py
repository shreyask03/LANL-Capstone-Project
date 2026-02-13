from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint_names = LaunchConfiguration("joint_names")
    gpio_pins = LaunchConfiguration("gpio_pins")
    angle_min_rad = LaunchConfiguration("angle_min_rad")
    angle_max_rad = LaunchConfiguration("angle_max_rad")
    pulse_min_us = LaunchConfiguration("pulse_min_us")
    pulse_max_us = LaunchConfiguration("pulse_max_us")
    pigpio_host = LaunchConfiguration("pigpio_host")
    pigpio_port = LaunchConfiguration("pigpio_port")
    publish_joint_states = LaunchConfiguration("publish_joint_states")

    return LaunchDescription(
        [
            DeclareLaunchArgument("joint_names", default_value="['joint1','joint2','joint3','joint4']"),
            DeclareLaunchArgument("gpio_pins", default_value="[12,13,18,19]", description="BCM pin numbers"),
            DeclareLaunchArgument("angle_min_rad", default_value="[-2.6,-2.0,-2.6,-2.6]"),
            DeclareLaunchArgument("angle_max_rad", default_value="[2.6,2.6,2.6,2.6]"),
            DeclareLaunchArgument("pulse_min_us", default_value="700.0"),
            DeclareLaunchArgument("pulse_max_us", default_value="2300.0"),
            DeclareLaunchArgument("pigpio_host", default_value="", description="Leave empty for localhost"),
            DeclareLaunchArgument("pigpio_port", default_value="8888"),
            DeclareLaunchArgument("publish_joint_states", default_value="true"),
            Node(
                package="arm_pwm_bridge",
                executable="joint_traj_to_pwm",
                output="screen",
                parameters=[
                    {
                        "joint_names": joint_names,
                        "gpio_pins": gpio_pins,
                        "angle_min_rad": angle_min_rad,
                        "angle_max_rad": angle_max_rad,
                        "pulse_min_us": ParameterValue(pulse_min_us, value_type=float),
                        "pulse_max_us": ParameterValue(pulse_max_us, value_type=float),
                        "pigpio_host": pigpio_host,
                        "pigpio_port": ParameterValue(pigpio_port, value_type=int),
                        "publish_joint_states": publish_joint_states,
                    }
                ],
            ),
        ]
    )
