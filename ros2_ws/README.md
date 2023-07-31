# Ros2 workspace

## Content

### src/Universal_Robots_ROS2_Description
Modified description package [Owner avatar
Universal_Robots_ROS2_Description
](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)

**Modificiations**
- Add vacuum gripper model

### src/Universal_Robots_ROS2_Driver
Humble branch of [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)

**Modificiations**
- No modificiations, git submodule

### src/ur_moveit_demo
- Simple paletization code for Moveit2
- Modified launch files for UR Moveit2 stack
- Launch file for paletization demo

# Building 

```
 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

## Running 

### Robot manipulation

```
ros2 launch ur_moveit_demo moveit.launch.py 
```

### Paletization demo
```
ros2 launch ur_moveit_demo palletization_demo.launch.py
```