# ROS 2 workspace

This ROS 2 workspace should be built and sourced before building or running the simulation project. Make sure to always source this workspace before running the simulator, as it won't work properly without it.

## Content

### src/Universal_Robots_ROS2_Description
Modified package of [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).

**Modifications**
- Added vacuum gripper model.

### src/Universal_Robots_ROS2_Driver
Branch of [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) for current $ROS_DISTRO, as a submodule.

### src/ur_palletization
- Simple palletization code for Moveit2.
- Modified launch files for UR Moveit2 stack.
- Launch file for palletization demo.
- Uses pilz_industrial_motion_planner.

### src/otto600_description src/otto1500_description
- Contains all urdf and mesh files for OTTO AMRs.

### src/otto_deliberation
- Autonomy for OTTO AMRs.
- Manages tasks, and sends commands to `otto_fleet_nav` and `blind_path_follower`.
- Contains task description configurations.

### src/otto_fleet_nav
- Nav2 for OTTO AMRs.
- Spawns and controls robots' movements.

### src/global_path_lock
- Manages access to paths that only one robot should take (e.g. loading, wrapping).
- Provides a simple locking ROS 2 service.

### src/blind_path_follower
- Steers robots during docking. It is here due to the fact that nav2 stack alone was not easy to tame for such tight approaches.

### src/roscon2023_demo
- Combines all launch files into one.
- Contains the configuration for all of the robots. You can change configuration files in this package to run a different setup.

## Changing the scene
### Changing the number of robots
To change the number of robots simply choose a different configuration file. For example, to spawn 8 robots instead of 4 use:
```bash
ros2 launch roscon2023_demo ROSCon2023Demo.launch.py ROS2Con2023Config:=<full_path_to>/ROSCon2023Config_8.yaml
```
> **_NOTE:_** Replace the `<full_path_to>/ROSCon2023Config_8.yaml` with the full path to your desired file.

There are multiple configuration files available. The files `ROSCon2023Config_2/4/8.yaml` are prepared for the smaller scene (`DemoLevel1.prefab`), while `ROSCon2023Config_900.yaml` should be used with the bigger scene (`DemoLevel2.prefab`).

You can also create a custom configuration file or modify the config by adding a new robot to the fleet:
```yaml
  - robot_name: otto600
    robot_namespace: otto_1
    lane: Line1
    tasks_config_file: config/otto600_tasks_left.yaml
    nav2_param_file: otto600Params.yaml
    position: 
      x: -30.0
      y: 2.0
      z: 0.1
```
Robots must have different namespaces to work and to be assigned a lane which is defined in the O3DE Editor (see "Line X" entities). You can also change the robots' spawn position, tasks configuration file, and Nav2 parameters file.

### Modifying the UR arms
To change the UR arms configuration modify the  `src/roscon2023_demo/config/ROSCon2023Config.yaml`, here is an example:
```yaml 
  - namespace: ur1
    num_of_boxes: 18
    launch_rviz: false
```
The arms should have the same namespace as in the O3DE Editor. These namespaces must be unique. You can also change the amount of boxes to be placed on the pallet (a maximum of 18 is supported).

### Modifying the robots' tasks
To change the task modify or add the `src/otto_deliberation/config/<nameoftasks>.yaml` file. Sample configuration can be found in `src/otto_deliberation/config/otto600_tasks_left.yaml`
```yaml
/**:
    ros__parameters:
      tasks : ["idle", "Path1"]
      lifter_tasks : [""]
      pre_task_delays : [0.0, 0.0]
      post_task_delays : [0.0, 0.0]
      dummy_tasks: ["idle"]
      blind_tasks: ["Path1"]
      blind_tasks_reverse: [""]
      cargo_load_tasks: [""]
      cargo_unload_tasks: [""]
      task_acquire_lock: [""]
      task_release_lock: [""]
```
- tasks: List of all robot tasks.
- lifter_tasks: List of tasks where the robot lifter should be in the raised position
- pre_task_delays/post_task_delays: Delay in seconds before/after task is completed. (must be the same length as the tasks list)
- dummy_tasks: Tasks that do not have a path associated with them.
- blind_tasks: Tasks which use the blind_path_follower robot controller (others use Nav2)
- blind_tasks_reverse: Blind tasks to be done in reverse
- cargo_load_tasks: Tasks where the cargo is loaded onto the robot
- cargo_unload_tasks: Tasks where the cargo is unloaded from the robot
- task_acquire_lock: Tasks where a lock is acquired (prevents robots from using the same path). Each lock must be later released
- task_release_lock: Tasks where a lock is released.

## Warning: demo code
Code quality is "demo", so be careful when using it for building serious projects. This codebase was only meant to enable the demonstration of O3DE capabilities.

