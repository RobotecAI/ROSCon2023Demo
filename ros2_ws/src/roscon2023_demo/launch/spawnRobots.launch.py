

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def launch_setup(context, *args, **kwargs):

    config_file_arg = LaunchConfiguration("ROS2Con2023Config")
    config_file = config_file_arg.perform(context)

    robots = []

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

    nodes_to_start = []

    spawner_group = GroupAction([
        Node(
            package='otto_fleet_nav',
            executable='robot_spawner',
            output='screen',
            parameters= [
                {"robot_name": robot["name"]},
                {"robot_namespace": robot["namespace"]},
                {"robot_initial_position": [robot["x_pose"], robot["y_pose"], robot["z_pose"]]}
            ]
        )
        for robot in robots
    ])

    nodes_to_start.append(spawner_group)

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    ROSCon2023Params = get_package_share_directory('roscon2023_demo')

    declared_arguments.append(
        DeclareLaunchArgument(
            'ROS2Con2023Config',
            default_value=os.path.join(ROSCon2023Params, 'config', 'ROSCon2023Config_8.yaml'),
            description='Full path to the ROS2 parameters file to use for all robot launched nodes'
        )
    )

    ld = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

    return ld
