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
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)

    robot_description_content = parse_xacro(
        os.path.join(get_package_share_directory("dual_rm_75_description"), "urdf", "dual_rm_75.urdf.xacro"),
        context=context,
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

    robot_controllers = os.path.join(this_package_share_directory, "config", "controllers.yaml")

    launch_entities.append(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_controllers,
                # {"robot_description": robot_description_content},  # Needed by admittance_controller
            ],
            output="screen",
            remappings=[
                ("controller_manager/robot_description", "robot_description"),
            ],
        )
    )

    initial_controller_states = {
        "joint_state_broadcaster": "active",
        "left_arm_controller": "active",
        "right_arm_controller": "active",
    }

    active_controllers = [key for key, value in initial_controller_states.items() if value == "active"]
    inactive_controllers = [key for key, value in initial_controller_states.items() if value == "inactive"]

    # Activate controllers
    if active_controllers:
        launch_entities.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=active_controllers,
            )
        )

    # Inactive controllers
    if inactive_controllers:
        launch_entities.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["--inactive"] + inactive_controllers,
            )
        )

    rviz_config_file = PathJoinSubstitution([this_package_share_directory, "rviz", "rm_driver.rviz"])

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
            "use_fake_hardware",
            description="Use fake hardware for testing purposes",
            default_value="false",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
