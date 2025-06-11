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

    robot_description_content = parse_xacro(
        os.path.join(this_package_share_directory, "urdf", f"rm.urdf.xacro"),
        context=context,
        name=robot_name,
        rm_type=rm_type,
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

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
