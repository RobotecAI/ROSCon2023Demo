import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration("namespace")
    assigned_lane = LaunchConfiguration("assigned_lane")

    nodes_to_start = []

    task_descriptions = os.path.join(
        get_package_share_directory('otto_deliberation'),
        'config',
        'simple_test.yaml'
    )
         
    otto_deliberation = Node(
        package="otto_deliberation",
        executable="deliberation_otto_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {"assigned_lane": assigned_lane},
            task_descriptions
        ],
    )
    
    
    nodes_to_start.append(otto_deliberation)
    return nodes_to_start


def generate_launch_description():


    namespace = DeclareLaunchArgument("namespace")

    assigned_lane = DeclareLaunchArgument("assigned_lane")

    declared_arguments = [namespace, assigned_lane]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
