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

### RViz2 
Run the command ```ros2 launch ur_moveit_demo rviz.launch.py ur_namespace:=ur1```. This will create an RViz2 instance visualizing one arms movements. Change the ```ur_namespace``` to ```ur2``` and add ```rviz_config_file:=view_robot_moveit_ur2.rviz``` to visualize the second arm.

### MoveIt
To launch MoveIt for a single arm run the command ```ros2 launch ur_moveit_demo mtc.launch.py ur_namespace:=ur1```. This will launch MoveIt for only one arm. To change the arm simply change ```ur_namespace``` from ```ur1``` to ```ur2```.  

Now the program controlling the arm waits for an ROS2 Action. To start the palleaziation launch ```ros2 action send_goal /ur1/MTC ur_moveit_demo_msg/action/Mtc "{num_of_boxes: 3}"```. This requires that a pallet is in front of the robotic arm.  
Change ```ur1``` to ```ur2``` to control the second arm. ```num_of_boxes``` is a parameter which specifies the amount of boxes to be placed on the pallet. The maximum number of boxes supported is 12.  

