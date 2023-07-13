# ROS 2 Workspace
This workspace consists of modified: [Universal Robots Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and [Universal Robots Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) repositories.

## Build
To build this workspace the following dependencies are required:  
```sudo apt install python3-colcon-common-extensions python3-vcstool```
Run the rosedep update and install to download and install all dependant packages:
```bash
rosdep update
rosdep install --ignore-src --from-paths src -y
```
Build the workspace using colcon:
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Source the workspace:
```
source install/setup.bash
```