# O3DE multi-robot demo

## Goal

This project serves the purpose to demonstrate [O3DE](https://www.o3de.org/) and its simulation capabilities [Interactivity and simulation](https://www.docs.o3de.org/docs/user-guide/interactivity/).
This project demonstrates an example of application complex cooperative ROS 2 application.
The integration is realized through [ROS 2 Gem](https://github.com/o3de/o3de-extras/blob/development/Gems/ROS2).

## How does it look like
<img src="media/view1.png" width="640">
<img src="media/view4.png" width="640">
<img src="media/view2.png" width="640">
<img src="media/view3.png" width="640">

## The project includes
- **Scenery** created using a [Warehouse project template](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/#ros-2-project-templates)
- **Robotic Arms** imported using [URDF description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) provided by Universal Robotics for their UR10 collaborative robot .  
- **AMRs** instantiated prefabs of OTTO600 and OTTO1500 robots from [OTTO Motors](https://ottomotors.com/).
- **Boxes** that are transported using conveyor belts and are to be palletized

## Platforms

The project runs on Ubuntu 22.04 with ROS 2 Humble or ROS 2 Iron.

💡 ***Note:*** This demo is **not supported on Windows!** 

## O3DE

1. Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make
   sure that the system/hardware requirements are met.
2. Please follow the instructions
   to [set up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/).
3. **Use the `stabilization2310` branch**.

The following commands should prepare O3DE:

```bash
cd {$WORKDIR}
git clone --branch stabilization2310 --single-branch https://github.com/o3de/o3de.git
cd o3de
git lfs install
git lfs pull
python/get_python.sh
scripts/o3de.sh register --this-engine
```

## ROS 2 Gem and other gems

This project uses the [ROS 2 Gem](https://github.com/o3de/o3de-extras/blob/development/Gems/ROS2), [Warehouse assets Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseAssets) and [Warehouse automation Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseAutomation).
Please make sure to follow the installation guide
in the [Project Configuration](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/) file.
To learn more about how the Gem works check out
the [Concepts and Structures](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/concepts-and-components-overview/).

Note that the Gem instructions include the installation of ROS 2 with some additional packages. 

 **Use the `stabilization2310` branch**.
 **During build use `AZ_USE_PHYSX5:=ON`** to enable PhysX 5.1. It is essential for articulation. 

We assume that the directory with the project is ```${WORKDIR}```.  
Clone o3de-extras repo
```bash 
cd ${WORKDIR}
git clone https://github.com/o3de/o3de-extras
cd o3de-extras
git lfs install
git lfs pull
```
And register used Gems:
```bash 
cd ${WORKDIR}
./o3de/scripts/o3de.sh register --gem-path o3de-extras/Gems/WarehouseAssets
./o3de/scripts/o3de.sh register --gem-path o3de-extras/Gems/WarehouseAutomation
```

The Gems are open to your contributions!

## Project 

Install necessary packages from ROS 2:
```bash 
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-control-toolbox ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-nav-msgs
```

You need to build and source the ROS 2 workspace first as it contains messages that the simulator uses to communicate.  
This workspace contains submodules that need to be pulled first.
```bash
cd ${WORKDIR}/ROSCon2023Demo/ros2_ws
git submodule init
git submodule update
```
Now install all dependencies of submodules.  
```bash
sudo apt install python3-colcon-common-extensions python3-vcstool
rosdep update
rosdep install --ignore-src --from-paths src/Universal_Robots_ROS2_Driver -y
sudo apt-get install ros-humble-moveit-servo ros-humble-moveit-visual-tools
```
Now build and source the workspace.
```bash
cd ${WORKDIR}/ROSCon2023Demo/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
The source command needs to be done in the same console where you build and run O3DE.

Assuming that [project's repo](https://github.com/RobotecAI/ROSCon2023Demo) was cloned to `{$WORKDIR}`:
```bash
cd {$WORKDIR}/ROSCon2023Demo/Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5:=ON
cmake --build build/linux --config profile --target Editor ROSCon2023Demo.Assets 
```
To launch the built project:
```bash
cd {$WORKDIR}/ROSCon2023Demo
./build/linux/bin/profile/Editor
```

## Running the simulation
Open the level: ```DemoLevel1.prefab```.  
Launch the O3DE simulation by clicking ```CTRL + G``` or by clicking the launch arrow next to the ```Play Controls``` in the top right corner.  
Now go to the ```ros2_ws``` folder and run the all ros2 packages.
```bash
cd {$WORKDIR}/ROSCon2023Demo/ros2_ws
source install/setup.bash
ros2 launch roscon2023_demo ROSCon2023Demo.launch.py
```
In a few seconds, the robots should spawn and start moving.  
For a more in-depth explanation see the [ros2_ws/README.md](ros2_ws/README.md).

 
