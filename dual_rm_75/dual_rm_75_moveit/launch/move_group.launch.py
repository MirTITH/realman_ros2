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
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context: LaunchContext, *args, **kwargs):
    launch_entities = []

    this_package_share_directory = get_this_package_share_directory(context)
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_semantic_file = os.path.join(
        get_package_share_directory("dual_rm_75_moveit_autogen"),
        "config",
        "dual_rm_75.srdf",
    )

    moveit_params = (
        MoveItConfigsBuilder(robot_name="dual_rm_75", package_name="dual_rm_75_moveit_autogen")
        .robot_description_semantic(robot_description_semantic_file)
        .planning_scene_monitor(publish_robot_description_semantic=True)
        .to_moveit_configs()
        .to_dict()
    )
    # We do not pass the robot_description parameter directly to move_group
    # Then the move_group node will obtain it from appropriate topic
    moveit_params.pop("robot_description")

    launch_entities.append(
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_params,
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    rviz_config_file = PathJoinSubstitution([this_package_share_directory, "rviz", "moveit.rviz"])
    launch_entities.append(
        Node(
            package="rviz2",
            executable="rviz2",
            # name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_params,  # Rviz2 Seems to require these parameters. Otherwise the InteractiveMarker will not show up.
                {"use_sim_time": use_sim_time},
            ],
        )
    )

    return launch_entities


def generate_launch_description():
    declared_arguments = []

    add_use_sim_time(declared_arguments, default_value="false")

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
