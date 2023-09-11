from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
        
    moveit_config = MoveItConfigsBuilder("ur20", package_name="UR20_moveit_config").to_moveit_configs()
    print (moveit_config)
    return generate_moveit_rviz_launch(moveit_config)
