

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def launch_setup(context, *args, **kwargs):
    palletization_dir = os.path.join(get_package_share_directory("ur_palletization"), 'launch')
    path_lock_dir = os.path.join(get_package_share_directory("global_path_lock"), "launch")
    otto_fleet_nav_dir = os.path.join(get_package_share_directory("otto_fleet_nav"), "launch")
    deliberation_dir = os.path.join(get_package_share_directory("otto_deliberation"), "launch")
    roscon2023demo_dir = os.path.join(get_package_share_directory("roscon2023_demo"), "launch")

    config_file_arg = LaunchConfiguration("ROS2Con2023Config")
    use_rviz = LaunchConfiguration("use_rviz")
    start_arm_deliberation = LaunchConfiguration("start_arm_deliberation")
    spawn_amrs = LaunchConfiguration("spawn_amrs")
    start_amr_navigation = LaunchConfiguration("start_amr_navigation")
    config_file = config_file_arg.perform(context)

    robots = []
    arms = []

    with open(config_file, 'r') as f:
        configuration = yaml.safe_load(f)
        for robot in configuration["fleet"]:
            robots.append(
                {
                    "name": robot["robot_name"],
                    "namespace": robot["robot_namespace"],
                    "lane": robot["lane"],
                    "x_pose": robot["position"]["x"],
                    "y_pose": robot["position"]["y"],
                    "z_pose": robot["position"]["z"],
                    "tasks_config_file": robot["tasks_config_file"],
                }
            )
        if "arms" in configuration:
            for ur in configuration["arms"]:
                arms.append(
                    {
                        "namespace" : ur["namespace"],
                        "num_of_boxes" : ur["num_of_boxes"],
                        "launch_rviz": ur["launch_rviz"],
                    }
                )

    nodes_to_start = []

    moveIt_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(palletization_dir, 'ur_palletization.launch.py')),
            launch_arguments = {
                "ur_namespace" : ur["namespace"],
                "num_of_boxes" : str(ur["num_of_boxes"]),
                "launch_rviz" : str(ur["launch_rviz"]),
            }.items()
        ) for ur in arms
    ])

    deliberation_group = TimerAction(
        period = 15.,
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(deliberation_dir, 'otto_deliberation.launch.py')),
                launch_arguments = {
                    "namespace" : robot["namespace"],
                    "assigned_lane" : robot["lane"],
                    "tasks_config_file" : os.path.join(get_package_share_directory("otto_deliberation"), robot["tasks_config_file"]),
                }.items()
            ) for robot in robots
        ],
        condition=IfCondition(LaunchConfiguration('start_arm_deliberation'))
    )

    path_lock = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(path_lock_dir, "global_path_lock.launch.py")
        ),
        launch_arguments = {
            "fleet_config_path": config_file_arg
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_arm_deliberation'))
    )

    otto_fleet_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(otto_fleet_nav_dir, "otto_fleet_nav_launch.py")
        ),
        launch_arguments = {
            "use_rviz": use_rviz,
            "fleet_config_path": config_file_arg,
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_amr_navigation'))
    )

    spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roscon2023demo_dir, "spawnRobots.launch.py")
        ),
        launch_arguments = {
            "fleet_config_path": config_file_arg,
        }.items(),
        condition=IfCondition(LaunchConfiguration('spawn_amrs'))
    )

    nodes_to_start.append(spawner)
    nodes_to_start.append(path_lock)
    nodes_to_start.append(moveIt_group)
    nodes_to_start.append(otto_fleet_nav)
    nodes_to_start.append(deliberation_group)

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    ROSCon2023Params = get_package_share_directory('roscon2023_demo')

    declared_arguments.append(
        DeclareLaunchArgument(
            'ROS2Con2023Config',
            default_value=os.path.join(ROSCon2023Params, 'config', 'ROSCon2023Config_4.yaml'),
            description='Full path to the ROS2 parameters file to use for all robot launched nodes'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="False",
            description="Launch rviz with nav2"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "spawn_amrs",
            default_value="True",
            description="Spawn AMRs"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_amr_navigation",
            default_value="True",
            description="Start AMRs with navigation"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "start_arm_deliberation",
            default_value="True",
            description="Use deliberation system for ARMs"
        )
    )
    ld = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

    return ld
