from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("rm", package_name="rm_75_moveit_autogen").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
