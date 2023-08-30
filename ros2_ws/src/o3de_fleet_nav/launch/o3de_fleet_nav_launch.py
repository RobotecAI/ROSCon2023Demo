# Copyright (c) 2018 Intel Corporation
# Copyright (c) Contributors to the Open 3D Engine Project.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the launch directory
    o3de_fleet_nav_dir = get_package_share_directory('o3de_fleet_nav')
    o3de_launch_dir = os.path.join(o3de_fleet_nav_dir, 'launch')

    fleet_config_file = os.path.join(o3de_fleet_nav_dir, 'config', 'fleet_config.yaml')

    robots = []
    with open(fleet_config_file, 'r') as f:
        configuration = yaml.safe_load(f)
        for robot in configuration["fleet"]:
            robots.append(
                {
                    "name": robot["robot_name"],
                    "namespace": robot["robot_namespace"],
                    "x_pose": robot["position"]["x"],
                    "y_pose": robot["position"]["y"],
                    "z_pose": robot["position"]["z"]
                }
            )

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    distro = os.getenv('ROS_DISTRO')

    # Declare the launch arguments
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(o3de_fleet_nav_dir, 'maps', 'video_level.yaml'),
        description='Full path to map file to load')

    declare_robot_params_file_cmd = DeclareLaunchArgument(
        'robot_params_file',
        default_value=os.path.join(o3de_fleet_nav_dir, 'params', distro, 'nav2_multirobot_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all robot launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='True',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(o3de_fleet_nav_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration("robot_params_file")
        
        configured_tree = ReplaceString(
            source_file=os.path.join(o3de_fleet_nav_dir, 'behaviour_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'),
            # source_file="/home/kacper/ROSCon2023Demo/ros2_ws/src/o3de_fleet_nav/params/navigate_through_poses_w_replanning_and_recovery.xml",
            replacements={
                'robot_namespace' : robot['namespace']
            }
        )

        configured_params = ReplaceString(
            source_file=params_file,
            replacements={
                '<robot_name>': robot['name'],
                '<robot_namespace>': robot['namespace'],
                '<robot_initial_pose_x>': str(robot['x_pose']),
                '<robot_initial_pose_y>': str(robot['y_pose']),
                '<robot_initial_pose_z>': str(robot['z_pose']),
                '<nav_through_poses_xml>': configured_tree
            })
        

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                        os.path.join(o3de_launch_dir, 'o3de_rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={
                                  'namespace': TextSubstitution(text=robot['namespace']),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(o3de_launch_dir,
                                                           'o3de_nav_launch.py')),
                launch_arguments={'namespace': robot['namespace'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': configured_params,
                                  'autostart': autostart,
                                  'use_rviz': 'False', }.items()),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['namespace']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['namespace'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['namespace'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['namespace'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['namespace'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld