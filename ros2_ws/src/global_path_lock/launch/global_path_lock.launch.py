

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):

    nodes_to_start = []

    path_description = os.path.join(
        get_package_share_directory('global_path_lock'),
        'config',
        'paths.yaml'
    )

    orchestration = Node(
        name="global_path_lock",
        package="global_path_lock",
        executable="global_path_lock",
        output="screen",
        parameters=[
            path_description
        ],
    )

    nodes_to_start.append(orchestration)


    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
