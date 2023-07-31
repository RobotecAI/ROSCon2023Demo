# O3DE multi-robot demo

## Goal

This project serves the purpose to demonstrate [O3DE](https://www.o3de.org/) and its simulation capabilities [Interactivity and simulation](https://www.docs.o3de.org/docs/user-guide/interactivity/).
This project demonstrates an example of application complex cooperative ROS 2 application.
The integration is realized through [ROS 2 Gem](https://github.com/o3de/o3de-extras/blob/development/Gems/ROS2).

## How does it look like
<img src="media/view1.png" width="640">
<img src="media/view2.png" width="640">

## The project includes
- **Scenery** created using a [Warehouse project template](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/#ros-2-project-templates)
- **Robotic Arms** imported using [URDF description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) provided by Universal Robotics for their UR10 collaborative robot .  
- **AMRs** instantiated prefabs of Proteus AMR robot from [ProteusRobot Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/ProteusRobot)
- **Boxes** that are transported using conveyor belts and are to be palletized

## Platforms

The project runs on Ubuntu 22.04 with ROS 2 Humble or ROS 2 Iron.

💡 ***Note:*** This demo is **not supported on Windows!** 

## O3DE

1. Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make
   sure that the system/hardware requirements are met.
2. Please follow the instructions
   to [set up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/).
3. **Use the `development` branch**.

The following commands should prepare O3DE:

```
~$ git clone --branch development --single-branch https://github.com/o3de/o3de.git
~$ cd o3de
~/o3de$ git lfs install
~/o3de$ git lfs pull
~/o3de$ python/get_python.sh
~/o3de$ scripts/o3de.sh register --this-engine
```

## ROS 2 Gem

This project uses the [ROS 2 Gem](https://github.com/o3de/o3de-extras/blob/development/Gems/ROS2).
Please make sure to follow the installation guide
in [Project Configuration](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/) file.
To learn more about how the Gem works check out
the [Concepts and Structures](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/concepts-and-components-overview/).

Note that the Gem instructions include the installation of ROS 2 with some additional packages. 

 **Use the `development` branch**.
 **During build use `AZ_USE_PHYSX5:=ON`** to enable PhysX 5.1. It is essential for articulations. 

The Gem is open to your contributions!

## Universal Robots ROS2 Driver and ROS2 workspace

Follow installation [Build from source](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble#build-from-source) section of [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
The modified URDF of the UR10 arm is provided in ros2 workspace ``

### Building ROS2 workspace

To build this workspace the following dependencies are required:  
```sudo apt install python3-colcon-common-extensions python3-vcstool```
Run the rosedep update and install to download and install all dependant packages:
```bash
rosdep update
rosdep install --ignore-src --from-paths src -y
```
Build the workspace using colcon:
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
```
Source the workspace:
```
source install/setup.bash
```
 
