from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_pwm_bridge = LaunchConfiguration("start_pwm_bridge")
    pwm_channels = LaunchConfiguration("pwm_channels")
    mavlink_connection = LaunchConfiguration("mavlink_connection")
    target_system = LaunchConfiguration("target_system")
    target_component = LaunchConfiguration("target_component")

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_bringup"), "launch", "ros2_control.launch.py"])
        ),
        launch_arguments={"controllers_file": controllers_file, "use_sim_time": use_sim_time}.items(),
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_launch"), "launch", "keyboard_teleop.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    pwm_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("arm_pwm_bridge"), "launch", "pwm_bridge.launch.py"])
        ),
        launch_arguments={
            "pwm_channels": pwm_channels,
            "mavlink_connection": mavlink_connection,
            "target_system": target_system,
            "target_component": target_component,
            "publish_joint_states": "false",
        }.items(),
        condition=IfCondition(start_pwm_bridge),
    )

    controllers_default = PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="ros2_control controller configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "start_pwm_bridge",
                default_value="true",
                description="Start the JointTrajectory->PWM bridge alongside teleop",
            ),
            DeclareLaunchArgument("pwm_channels", default_value="[9,10,11,12]"),
            DeclareLaunchArgument("mavlink_connection", default_value="udp:127.0.0.1:14550"),
            DeclareLaunchArgument("target_system", default_value="1"),
            DeclareLaunchArgument("target_component", default_value="1"),
            bringup_launch,
            teleop_launch,
            pwm_bridge_launch,
        ]
    )
