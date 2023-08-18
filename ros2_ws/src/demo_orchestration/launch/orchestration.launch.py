

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):

    ur_namespace = LaunchConfiguration("ur_namespace")
    amr_namespaces = LaunchConfiguration("amr_namespaces")
    num_of_boxes = LaunchConfiguration("number_of_boxes")
    path_namespace = LaunchConfiguration("path_namespace")

    nodes_to_start = []

    for x in amr_namespaces.perform(context).strip('][').split(', '):
        nodes_to_start.append(Node(
            name="pathFollower",
            package="demo_orchestration",
            executable="path_follower",
            output="screen",
            parameters=[
                {"use_sim_time": True},
                {"ns": x},
            ],
        ))

    orchestration = Node(
        name="orchestration",
        package="demo_orchestration",
        executable="demo_orchestration",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"ur_namespace": ur_namespace},
            {"amr_namespaces": amr_namespaces},
            {"number_of_boxes": num_of_boxes},
            {"path_namespace": path_namespace}
        ],
    )

    nodes_to_start.append(orchestration)


    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_namespace",
            default_value='""',
            description="Namespace for the robot arm, useful for running multiple instances.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "amr_namespaces",
            default_value=[],
            description="Namespace for the amr, useful for running multiple instances.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "path_namespace",
            default_value='""',
            description="Namespace for the paths for the AMRs",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "number_of_boxes",
            default_value='""',
            description="Number of boxes to be placed on the pallet",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
