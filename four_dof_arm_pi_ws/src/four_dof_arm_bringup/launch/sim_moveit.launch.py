from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("four_dof_arm_bringup"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "controllers_file": controllers_file,
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("four_dof_arm_moveit_config"), "launch", "move_group.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "controllers_file": controllers_file,
            "start_rviz": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controllers_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("four_dof_arm_bringup"), "config", "ros2_controllers.yaml"]
                ),
                description="Controller configuration shared with MoveIt and Gazebo",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
            gazebo_launch,
            moveit_launch,
        ]
    )
