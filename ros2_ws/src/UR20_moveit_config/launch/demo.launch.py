import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_rsp_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur20", package_name="UR20_moveit_config").to_moveit_configs()

    # moveit_config["use_sim_time"] = True 

    
    ld = LaunchDescription()

     # Given the published joint states, publish tf for the robot links and the robot description
    rsp_node = Node(
         package="robot_state_publisher",
         executable="robot_state_publisher",
         respawn=True,
         output="screen",
         parameters=[
             moveit_config.robot_description,
             {
                 "use_sim_time": True,
                 "publish_frequency": 50.0,
             },
         ],
     )
    ld.add_action(rsp_node)

    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
         DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
         DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))
  
    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
  
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
  
    move_group_configuration = {
         "publish_robot_description_semantic": True,
         "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
         # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
         "capabilities": ParameterValue(
             LaunchConfiguration("capabilities"), value_type=str
         ),
         "disable_capabilities": ParameterValue(
             LaunchConfiguration("disable_capabilities"), value_type=str
         ),
         # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
         "publish_planning_scene": should_publish,
         "publish_geometry_updates": should_publish,
         "publish_state_updates": should_publish,
         "publish_transforms_updates": should_publish,
         "monitor_dynamics": False,
    }
    d =moveit_config.to_dict()
    d["use_sim_time"] = True
    move_group_params = [
        d,
         move_group_configuration,
    ]
  
    add_debuggable_node(
         ld,
         package="moveit_ros_move_group",
         executable="move_group",
         commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
         output="screen",
         parameters=move_group_params,
         extra_debug_args=["--debug"],
         # Set the display variable, in case OpenGL code is used internally
         additional_env={"DISPLAY": os.environ["DISPLAY"]},
    )


    #descriptor1 = generate_move_group_launch(moveit_config)
    #descriptor2 = generate_rsp_launch(moveit_config)
    descriptor3 = generate_moveit_rviz_launch(moveit_config)
    

    

    return LaunchDescription([ld,descriptor3])
