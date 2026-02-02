from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")
    controllers_file = LaunchConfiguration("controllers_file")

    robot_description_content = Command(
        [
            "xacro ",
            PathJoinSubstitution([FindPackageShare("four_dof_arm_description"), "urdf", "four_dof_arm.urdf.xacro"]),
            " controllers_file:=",
            controllers_file,
        ]
    )

    robot_description = {"robot_description": robot_description_content, "use_sim_time": use_sim_time}
    controllers_yaml = PathJoinSubstitution([FindPackageShare("four_dof_arm_bringup"), "config", "ros2_controllers.yaml"])

    # Launch Gazebo (gz-sim) via ros_gz_sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Bridge /clock from gz to ROS
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            "four_dof_arm",
            "-allow_renaming",
            "false",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", controllers_yaml],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
            DeclareLaunchArgument(
                "gz_args",
                default_value=TextSubstitution(text="-r -v 3 empty.sdf"),
                description="Arguments passed to gz sim (world, verbosity, headless, etc.)",
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value=controllers_yaml,
                description="Controller configuration shared with MoveIt",
            ),
            gazebo,
            robot_state_publisher_node,
            clock_bridge,
            spawn_entity,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=spawn_entity,
                    on_start=[joint_state_broadcaster_spawner, arm_controller_spawner],
                )
            ),
        ]
    )
