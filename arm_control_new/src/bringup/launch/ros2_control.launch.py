from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_path = PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "four_dof_arm.urdf.xacro"])
    controllers_default = PathJoinSubstitution([FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"])

    robot_description = {
        "robot_description": Command(["xacro ", description_path, " controllers_file:=", controllers_file]),
        "use_sim_time": use_sim_time,
    }

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager", "--param-file", controllers_file],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_default,
                description="ros2_control controller configuration",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation clock"),
            control_node,
            robot_state_publisher,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=control_node,
                    on_start=[joint_state_broadcaster_spawner, arm_controller_spawner],
                )
            ),
        ]
    )
