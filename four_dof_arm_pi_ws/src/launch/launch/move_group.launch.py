import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _load_yaml(package_name: str, relative_path: str):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r") as file:
        return yaml.safe_load(file)


def _sanitize(obj):
    if isinstance(obj, dict):
        return {k: _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, tuple):
        return [_sanitize(v) for v in obj]
    if isinstance(obj, list):
        return [_sanitize(v) for v in obj]
    return obj


def _load_text(package_name: str, relative_path: str):
    path = Path(get_package_share_directory(package_name)) / relative_path
    return path.read_text()


def launch_setup(context, *args, **kwargs):
    controllers_file = LaunchConfiguration("controllers_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    description_path = PathJoinSubstitution([FindPackageShare("arm_description"), "urdf", "four_dof_arm.urdf.xacro"])

    robot_description_content = Command(
        [
            "xacro ",
            description_path,
            " controllers_file:=",
            controllers_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content, "use_sim_time": use_sim_time}
    robot_description_semantic = {
        "robot_description_semantic": _load_text("arm_config", "config/four_dof_arm.srdf")
    }
    kinematics_yaml = _sanitize(_load_yaml("arm_config", "config/kinematics.yaml"))
    ompl_yaml = _sanitize(_load_yaml("arm_config", "config/ompl_planning.yaml"))
    joint_limits_yaml = _sanitize(_load_yaml("arm_config", "config/joint_limits.yaml"))
    moveit_controllers_yaml = _sanitize(_load_yaml("arm_config", "config/moveit_controllers.yaml"))
    initial_positions = _sanitize(_load_yaml("arm_config", "config/initial_positions.yaml"))

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 0.9,
        "trajectory_execution.allowed_goal_duration_margin": 0.2,
        "trajectory_execution.allowed_start_tolerance": 0.005,
    }

    planning_scene_monitor_params = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_yaml,
                joint_limits_yaml,
                moveit_controllers_yaml,
                initial_positions,
                ompl_yaml,
                trajectory_execution,
                planning_scene_monitor_params,
            ],
        ),
    ]

    if start_rviz.perform(context).lower() == "true":
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="moveit_rviz",
                output="screen",
                parameters=[robot_description, robot_description_semantic],
                arguments=[],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("arm_config"), "config", "ros2_controllers.yaml"]
                ),
                description="Controller configuration shared with ros2_control",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation clock if true"),
            DeclareLaunchArgument("start_rviz", default_value="true", description="Start RViz2 alongside move_group"),
            OpaqueFunction(function=launch_setup),
        ]
    )
