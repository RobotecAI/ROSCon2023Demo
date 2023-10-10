# Ros2 workspace

## Content

### src/Universal_Robots_ROS2_Description
Modified description package [Owner avatar
Universal_Robots_ROS2_Description
](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

**Modifications**
- Add vacuum gripper model

### src/Universal_Robots_ROS2_Driver
Humble branch of [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)

**Modifications**
- No modifications, git submodule

### src/ur_palletization
- Simple palletization code for Moveit2
- Modified launch files for UR Moveit2 stack
- Launch file for palletization demo
- Uses pilz_industrial_motion_planner

### src/otto600_description src/otto1500_description
- Contains all urdf and mesh files for OTTO AMRs

### src/otto_deliberation
- Autonomy for OTTO AMRs
- Manages tasks, sends commands to otto_fleet_nav and blind_path_follower
- Contains task description configurations

### src/otto_fleet_nav
- Nav2 for OTTO AMRs
- Spawns and controls robots movements

### src/global_path_lock
- Manages access to all paths that robots take
- Provides a path locking ROS2 service

### src/blind_path_follower
- Steers robots during docking

### src/roscon2023_demo
- Combines all launch files into one
- Contains the configuration for all of the robots

## Changing the scene
### Changing robots amount
To change the robots amount simply modify the ```src/roscon2023_demo/config/ROSCon2023Config.yaml``` by adding a new robot to the fleet:
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
Robots must have different namespaces to work and to be assigned a lane which is defined in the O3DE Editor. You can also change the robots spawn position, tasks configuration file and Nav2 parameters file.

### Modifying the UR arms
To change the UR arms configuration modify the  ```src/roscon2023_demo/config/ROSCon2023Config.yaml``` by adding a new UR arm:
```yaml 
  - namespace: ur1
    num_of_boxes: 18
    launch_rviz: false
```
The arms should have the same namespace as in the O3DE Editor. These namespaces must be unique. YOu can also change the amount of boxes to be placed on the pallet (maximum of 18 is supported).

### Modifying the robots tasks
To change task modify or adding the ```src/otto_deliberation/config/<nameoftasks>.yaml``` file. Example configuration can be found in ```src/otto_deliberation/config/otto600_tasks_left.yaml```
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

