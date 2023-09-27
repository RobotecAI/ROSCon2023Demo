

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, GroupAction, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def launch_setup(context, *args, **kwargs):

    blind_path_follower_dir = os.path.join(get_package_share_directory("blind_path_follower"), 'launch')
    mtc_dir = os.path.join(get_package_share_directory("ur_moveit_demo"), 'launch')
    orchestrator_dir = os.path.join(get_package_share_directory("demo_orchestration"), "launch")
    o3de_fleet_nav_dir = os.path.join(get_package_share_directory("o3de_fleet_nav"), "launch")
    deliberation_dir = os.path.join(get_package_share_directory("otto_deliberation"), "launch")

    config_file_arg = LaunchConfiguration("ROS2Con2023Config")
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
                    "z_pose": robot["position"]["z"]
                }
            )
        
        for ur in configuration["arms"]:
            arms.append(
                {
                    "namespace" : ur["namespace"],
                    "num_of_boxes" : ur["num_of_boxes"],
                    "launch_rviz": ur["launch_rviz"],
                }
            )

    nodes_to_start = []

    blind_path_followers_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
            launch_arguments = {
                "amr_namespace" : robot["namespace"],
            }.items()
        ) for robot in robots
    ])

    moveIt_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mtc_dir, 'mtc.launch.py')),
            launch_arguments = {
                "ur_namespace" : ur["namespace"],
                "num_of_boxes" : str(ur["num_of_boxes"]),
                "launch_rviz" : str(ur["launch_rviz"]),
            }.items()
        ) for ur in arms
    ])

    deliberation_group = TimerAction(
        period = 30.,
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(deliberation_dir, 'otto600_deliberation.launch.py')),
                launch_arguments = {
                    "namespace" : robot["namespace"],
                    "assigned_lane" : robot["lane"],
                }.items()
            ) for robot in robots
        ]
    )

    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orchestrator_dir, "orchestration.launch.py")
        ),
        launch_arguments = {
            "fleet_config_path": config_file_arg
        }.items()
    )

    o3de_fleet_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(o3de_fleet_nav_dir, "o3de_fleet_nav_launch.py")
        ),
        launch_arguments = {
            "use_rviz": "False",
            "fleet_config_path": config_file_arg,
        }.items()
    )

    nodes_to_start.append(orchestrator)
    # nodes_to_start.append(blind_path_followers_group)
    nodes_to_start.append(moveIt_group)
    nodes_to_start.append(o3de_fleet_nav)
    nodes_to_start.append(deliberation_group)

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    ROSCon2023Params = get_package_share_directory('roscon2023_demo')

    declared_arguments.append(
        DeclareLaunchArgument(
            'ROS2Con2023Config',
            default_value=os.path.join(ROSCon2023Params, 'config', 'ROSCon2023Config.yaml'),
            description='Full path to the ROS2 parameters file to use for all robot launched nodes'
        )
    )

    ld = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

    return ld
