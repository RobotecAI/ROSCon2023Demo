

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    ur_namespace = LaunchConfiguration("ur_namespace")

    orchestration = Node(
        name="orchestrationName",
        package="demo_orchestration",
        executable="demo_orchestration",
        output="screen",
        namespace=ur_namespace,
        parameters=[
        ],
    )

    nodes_to_start = [orchestration]


    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_namespace",
            default_value='""',
            description="Namespace for the robot, useful for running multiple instances.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
