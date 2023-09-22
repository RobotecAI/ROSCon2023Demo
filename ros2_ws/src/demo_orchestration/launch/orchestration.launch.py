

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    nodes_to_start = []

    orchestration = Node(
        name="orchestration",
        package="demo_orchestration",
        executable="demo_orchestration",
        output="screen",
    )

    nodes_to_start.append(orchestration)


    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
