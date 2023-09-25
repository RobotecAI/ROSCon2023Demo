

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction, GroupAction, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


# def launch_setup(context, *args, **kwargs):

    # ur_namespace = LaunchConfiguration("ur_namespace")
    # amr_namespace = LaunchConfiguration("amr_namespace")
    # num_of_boxes = LaunchConfiguration("number_of_boxes")
    # path_namespace = LaunchConfiguration("path_namespace")

    # blind_path_follower_dir = os.path.join(get_package_share_directory("blind_path_follower"), 'launch')

    # nodes_to_start = []

    # nodes_to_start.append(Node(
    #     name="pathFollower",
    #     package="blind_path_follower",
    #     executable="blind_path_follower",
    #     output="screen",
    #     parameters=[
    #         {"use_sim_time": True},
    #         {"robot_namespace": amr_namespace},
    #     ],
    # ))

    # blind_path_followers_group = GroupAction([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
    #         launch_arguments = {
    #             "namespace" : "otto_1",
    #             "assigned_lane" : "Line1",
    #         }
    #     ),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
    #         launch_arguments = {
    #             "namespace" : "otto_2",
    #             "assigned_lane" : "Line2",
    #         }
    #     ),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
    #         launch_arguments = {
    #             "namespace" : "otto_3",
    #             "assigned_lane" : "Line1",
    #         }
    #     ),
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
    #         launch_arguments = {
    #             "namespace" : "otto_4",
    #             "assigned_lane" : "Line2",
    #         }
    #     ),
    # ])

    # nodes_to_start.append(blind_path_followers_group)

    # return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    ROSCon2023Params = get_package_share_directory('ROSCon2023Demo')

    declared_arguments.append(
        DeclareLaunchArgument(
            'ROS2Con2023Params',
            default_value=os.path.join(ROSCon2023Params, 'params', 'ROSCon2023Params.yaml'),
            description='Full path to the ROS2 parameters file to use for all robot launched nodes'
        )
    )

    blind_path_follower_dir = os.path.join(get_package_share_directory("blind_path_follower"), 'launch')
    mtc_dir = os.path.join(get_package_share_directory("ur_moveit_demo"), 'launch')
    orchestrator_dir = os.path.join(get_package_share_directory("demo_orchestration"), "launch")
    o3de_fleet_nav_dir = os.path.join(get_package_share_directory("o3de_fleet_nav"), "launch")
    deliberation_dir = os.path.join(get_package_share_directory("otto_deliberation"), "launch")

    blind_path_followers_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
            launch_arguments = {
                "amr_namespace" : "otto_1",
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
            launch_arguments = {
                "amr_namespace" : "otto_2",
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
            launch_arguments = {
                "amr_namespace" : "otto_3",
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(blind_path_follower_dir, 'blind_path_follower.launch.py')),
            launch_arguments = {
                "amr_namespace" : "otto_4",
            }.items()
        ),
    ])

    moveIt_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mtc_dir, 'mtc.launch.py')),
            launch_arguments = {
                "ur_namespace" : "ur1",
                "num_of_boxes" : "18",
                "launch_rviz" : "false",
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mtc_dir, 'mtc.launch.py')),
            launch_arguments = {
                "ur_namespace" : "ur2",
                "num_of_boxes" : "18",
                "launch_rviz" : "false",
            }.items()
        ),
    ])

    deliberation_group = TimerAction(
        period = 10.,
        actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(deliberation_dir, 'otto_deliberation.launch.py')),
                launch_arguments = {
                    "namespace" : "otto_1",
                    "assigned_lane" : "Line1",
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(deliberation_dir, 'otto_deliberation.launch.py')),
                launch_arguments = {
                    "namespace" : "otto_2",
                    "assigned_lane" : "Line2",
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(deliberation_dir, 'otto_deliberation.launch.py')),
                launch_arguments = {
                    "namespace" : "otto_3",
                    "assigned_lane" : "Line1",
                }.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(deliberation_dir, 'otto_deliberation.launch.py')),
                launch_arguments = {
                    "namespace" : "otto_4",
                    "assigned_lane" : "Line2",
                }.items()
            ),
        ]
    )

    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orchestrator_dir, "orchestration.launch.py")
        )
    )

    o3de_fleet_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(o3de_fleet_nav_dir, "o3de_fleet_nav_launch.py")
        )
    )

    # ld = LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
    ld = LaunchDescription(declared_arguments)

    ld.add_action(orchestrator)
    ld.add_action(blind_path_followers_group)
    ld.add_action(moveIt_group)
    ld.add_action(o3de_fleet_nav)
    ld.add_action(deliberation_group)

    return ld
