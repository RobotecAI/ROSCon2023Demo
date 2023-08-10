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

### src/ur_moveit_demo
- Simple palletization code for Moveit2
- Modified launch files for UR Moveit2 stack
- Launch file for palletization demo

# Prerequists

Clone MoveIt task constructor.
```
git submodule init
git submodule update
```
Install vcs
```
sudo apt install python3-colcon-common-extensions python3-vcstool
```
Install necessary packages:
```
rosdep update
rosdep install --ignore-src --from-paths src -y
```

```
apt-get install ros-humble-moveit-servo
```

# Building 

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```

## Running 

### Robot manipulation

```
ros2 launch ur_moveit_demo moveit.launch.py 
```

### Paletization demo using Moveit Task Constructor
```
ros2 launch ur_moveit_demo mtc.launch.py
```
