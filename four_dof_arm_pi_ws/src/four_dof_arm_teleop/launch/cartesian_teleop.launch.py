from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    description_path = PathJoinSubstitution(
        [FindPackageShare("four_dof_arm_description"), "urdf", "four_dof_arm.urdf.xacro"]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                description_path,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
            Node(
                package="four_dof_arm_teleop",
                executable="keyboard_to_joy",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="four_dof_arm_teleop",
                executable="joy_to_pose_ik",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description["robot_description"],
                        "linear_scale": 0.05,
                        "angular_scale": 0.8,
                        "deadzone": 0.05,
                        "deadman_button": 4,
                        "publish_rate_hz": 5.0,
                        "group_name": "arm",
                        "arm_controller_ns": "arm_controller",
                        "ik_link": "tool0",
                        "planning_frame": "base_link",
                        "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                    }
                ],
            ),
        ]
    )
