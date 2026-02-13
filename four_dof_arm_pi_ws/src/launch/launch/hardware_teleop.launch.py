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
        launch_arguments={"publish_joint_states": "false"}.items(),
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
            bringup_launch,
            teleop_launch,
            pwm_bridge_launch,
        ]
    )
