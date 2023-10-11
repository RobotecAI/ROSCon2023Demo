

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    ur_namespace = LaunchConfiguration("ur_namespace")
    amr_namespace = LaunchConfiguration("amr_namespace")
    num_of_boxes = LaunchConfiguration("number_of_boxes")
    path_namespace = LaunchConfiguration("path_namespace")

    nodes_to_start = []

    nodes_to_start.append(Node(
        name="pathFollower",
        package="blind_path_follower",
        namespace=amr_namespace,
        executable="blind_path_follower",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    ))

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "amr_namespace",
            description="Namespace for the amr",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
