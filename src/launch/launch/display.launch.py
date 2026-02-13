from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "four_dof_arm.urdf.xacro"]),
            " controllers_file:=",
            controllers_file,
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content, "use_sim_time": use_sim_time}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"]),
                description="ros2_control controller config used by MoveIt and the hardware stack",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation clock if true"),
            robot_state_publisher_node,
            joint_state_publisher_node,
        ]
    )
