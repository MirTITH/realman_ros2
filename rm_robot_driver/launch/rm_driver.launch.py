import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_scripts import get_this_package_name, get_this_package_share_directory, add_use_sim_time, parse_xacro
from launch.launch_context import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def launch_setup(context: LaunchContext, *args, **kwargs):
    launch_entities = []

    this_package_share_directory = get_this_package_share_directory(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_name = LaunchConfiguration("robot_name").perform(context)
    rm_type = LaunchConfiguration("rm_type").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    description_file = LaunchConfiguration("description_file").perform(context)
    if not description_file:
        # If no description file is provided, use the default URDF file
        description_file = os.path.join(this_package_share_directory, "urdf", "rm_driver.urdf.xacro")

    robot_description_content = parse_xacro(
        description_file,
        context=context,
        name=robot_name,
        rm_type=rm_type,
        use_fake_hardware=use_fake_hardware,
    )

    launch_entities.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[
                {
                    "robot_description": robot_description_content,
                    "use_sim_time": use_sim_time,
                }
            ],
        )
    )

    launch_entities.append(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    rviz_config_file = PathJoinSubstitution([this_package_share_directory, "rviz", "view_robot.rviz"])

    launch_entities.append(
        Node(
            package="rviz2",
            executable="rviz2",
            # name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return launch_entities


def generate_launch_description():
    declared_arguments = []

    add_use_sim_time(declared_arguments, default_value="false")

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            description="Realman robot name",
            default_value="rm",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rm_type",
            description="Realman robot type",
            default_value="rm_65_6f",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            description="Use fake hardware for testing purposes",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            description="Robot description file path",
            default_value="",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
