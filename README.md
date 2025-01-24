# Robotized fulfillment center

An [Open 3D Engine (O3DE)]((https://www.o3de.org/)) multi-robot demo.

## The project

This project demonstrates O3DE use for a complex robotic simulation through integration with modern ROS 2 stacks: nav2 and MoveIt2.
You can learn more about the features of O3DE for robotics and how to get started in [O3DE documentation](https://www.docs.o3de.org/docs/user-guide/interactivity/).

### Levels
- **DemoLevel1**: 30x100 meters scene with 4 conveyor belts, and 4 robotic arms. Suitable for 4-8 AMRs.
- **DemoLevel2**: 90x100 meters, three times larger, and 12 robotic arms, suitable for 12-24 AMRs.
- **RobotsSuperShot**: a level showcasing 3D models, with several robots, a human, and a forklift. Some robots are not equipped with all components yet, but you are welcome to try and make them work!
- **RobotImportLevel**: a small enclosed space with a table, good for [importing your own robot](https://docs.o3de.org/docs/user-guide/interactivity/robotics/importing-robot/).
- **DemoStereo**: a level showcasing HIL setup with AMD KRIA and stereo cameras. Visit [KRIA depth demo](https://github.com/RobotecAI/kria_depth_demo) for more information.

### Detailed description
UR20 robot arms controlled by MoveIt2 with [pilz_industrial_motion_planner](https://moveit.picknik.ai/humble/doc/examples/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html?highlight=pilz#pilz-industrial-motion-planner).
Boxes are supplied by conveyor belts, which are implemented through spawning when below a certain number in an area.
UR20 arms are placing boxes based on a ground truth vision system, which means they look at the scene and that there is no error in pose measurement.
UR20 arms start working as soon as an immobile pallet is detected in their load area and will load a configurable number of boxes (up to 18, by default 18) on each pallet.

Pallets are moved around by robots modeled after OTTO 600.
Note that these AMRs do not use the software of real OTTO 600 and do not have the same sensors.
They can navigate thanks to front/back lidar sensors and operate cargo lifts.
OTTO 600 robots have assigned task loops, which are loading, wrapping, and delivering cargo to the other end of the warehouse.
They use a nav2 action server to realize their paths, and also a custom path follow solution for docking and unloading (for simplicity).
Note that robots follow their task independently but see and avoid each other and people walking around.
On the other hand, human workers don't see robots, as they use navigation through a Gem and only consider static scene objects.

### How does it look like
<img src="media/view1.png" width="640">
<img src="media/view4.png" width="640">
<img src="media/view2.png" width="640">
<img src="media/view3.png" width="640">

### The project includes
- **Scenery** initially created using a [Warehouse project template](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/#ros-2-project-templates), but many new models have been added.
- **Robotic Arms** imported using [URDF description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) provided by Universal Robotics for their UR20 collaborative robot. The prefabs are available in a separate Gem.
- **AMRs** instantiated prefabs of OTTO600 and OTTO1500 robots from [OTTO Motors](https://ottomotors.com/).
- **Boxes** that are transported using conveyor belts from [Warehouse Automation Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseAutomation) and palletized.

## Requirements

### Platforms
The project runs on Ubuntu 22.04 with ROS 2 Humble or ROS 2 Iron.

💡 ***Note:*** This demo is **not supported on ROS 2 Jazzy!**
💡 ***Note:*** This demo is **not supported on Windows!**

### Hardware
The demo is rather demanding, as it aims to show what is possible. Minimum specs are not determined, but we ran it on:
- AMD Radeon RX 6800 / NVIDIA GeForce RTX 3070 GPU (8 GB).
- AMD Ryzen 9 5900HX CPU / Intel i7-11800H.
- 64 GB RAM.

For more FPS, a larger scene, and more robots, we used:
- AMD Radeon RX 7900 XT / NVIDIA RTX 3080 Ti (or better) GPU (16 GB).
- AMD Ryzen 9 7950X / Intel i7-12900KF (24 cores) CPU.
- 64 GB RAM.

## Docker container environment

Follow the instructions in [./Docker/README](./Docker/README.md) file to build and run the project using *docker* virtualization.

## Project Setup

### ROS 2 middleware
This project should be used with the `rmw_cyclonedds_cpp` as the ROS 2 middleware.
[MoveIt2 does not recommend usage of the default RMW](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#switch-to-cyclone-dds) and as it is a part of this project using the default RMW will not work.

Install the CycloneDDS RMW by installing its package:
```bash
sudo apt install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
```
After the installation add this command to your `.bashrc` or equivalent file.
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
This will change the default RMW implementation to CycloneDDS.
Source your new configuration:
```bash
source ~/.bashrc
```
> ***Note:*** The ROS2 daemon may need to be restarted to use the CycloneDDS RMW. Use `ros2 daemon stop` and `ros2 daemon start` to restart the daemon.

### O3DE
1. Refer to the [O3DE System Requirements](https://www.o3de.org/docs/welcome-guide/requirements/) documentation to make sure that the system/hardware requirements are met.
2. Please follow the instructions to [set up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/).
3. This project was tested on O3DE 2409.1. **`o3de` 2409.1 and `o3de-extras` 2409.1 are recommended versions**, but the newer point-releases should work.

The following commands should prepare O3DE (assuming that the project repository is cloned into `${WORKDIR}`):

```bash
cd ${WORKDIR}
git clone --branch 2409.1 --single-branch --depth 1 https://github.com/o3de/o3de.git
cd o3de
git lfs install
git lfs pull
python/get_python.sh
scripts/o3de.sh register --this-engine
```

### ROS 2 Gem and other Gems
This project uses the following Gems:
- [ROS 2 Gem](https://github.com/o3de/o3de-extras/blob/development/Gems/ROS2)
- [Warehouse assets Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseAssets) 
- [Warehouse automation Gem](https://github.com/o3de/o3de-extras/tree/development/Gems/WarehouseAutomation)
- [HumanWorker Gem](https://github.com/RobotecAI/o3de-humanworker-gem)
- [UR10 and UR20 Robots Gem](https://github.com/RobotecAI/o3de-ur-robots-gem)
- [OTTO 600 and OTTO 1500 Robots Gem](https://github.com/RobotecAI/o3de-otto-robots-gem)
- [Additional warehouse assets](https://github.com/RobotecAI/robotec-warehouse-assets)
- [Additional generic assets](https://github.com/RobotecAI/robotec-generic-assets)


Please make sure to follow the installation guide in the [Project Configuration](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/) file up until the creation of a new Project.

To learn more about how the Gem works check out the [Concepts and Structures](https://www.docs.o3de.org/docs/user-guide/interactivity/robotics/concepts-and-components-overview/).

Note that the Gem instructions include the installation of ROS 2 with some additional packages.

```bash
cd ${WORKDIR}
git clone --branch 2409.1 --single-branch --depth 1 https://github.com/o3de/o3de-extras
cd o3de-extras
git lfs install
git lfs pull
```
And register required Gems:
```bash
cd ${WORKDIR}
./o3de/scripts/o3de.sh register --gem-path o3de-extras/Gems/ROS2
./o3de/scripts/o3de.sh register --gem-path o3de-extras/Gems/WarehouseAssets
./o3de/scripts/o3de.sh register --gem-path o3de-extras/Gems/WarehouseAutomation
```

Clone and register the remaining Gems. Use version `1.0.0` for all Gems:
```bash
cd ${WORKDIR}
git clone --branch 2.0.0 --single-branch --depth 1 https://github.com/RobotecAI/o3de-humanworker-gem.git
git clone --branch 2.0.0 --single-branch --depth 1 https://github.com/RobotecAI/o3de-ur-robots-gem.git
git clone --branch 2.0.0 --single-branch --depth 1 https://github.com/RobotecAI/o3de-otto-robots-gem
git clone https://github.com/RobotecAI/robotec-warehouse-assets.git 
git clone https://github.com/RobotecAI/robotec-generic-assets.git 
git clone https://github.com/RobotecAI/robotec-o3de-tools.git
./o3de/scripts/o3de.sh register --gem-path o3de-humanworker-gem
./o3de/scripts/o3de.sh register --gem-path o3de-ur-robots-gem
./o3de/scripts/o3de.sh register --gem-path o3de-otto-robots-gem
./o3de/scripts/o3de.sh register --all-gems-path robotec-warehouse-assets
./o3de/scripts/o3de.sh register --all-gems-path robotec-generic-assets
./o3de/scripts/o3de.sh register --gem-path robotec-o3de-tools/Gems/ROS2ScriptIntegration
```

The Gems are open to your contributions!

### RGL Gem (Optional)
Optionally, especially when intending to run more robots or change their lidar sensors to higher resolution ones, you can enable and use Robotec GPU Lidar Gem (RGL Gem).
Please follow the instructions in the [RGL Gem repository](https://github.com/RobotecAI/o3de-rgl-gem), register it (see above) and enable it within the project.
After that, change the OTTO 600 prefab so that both front and back lidars use the GPU lidar (use combo box to select it).

### ROS 2 packages
Make sure to install the necessary ROS 2 packages.
```bash
sudo apt install ros-${ROS_DISTRO}-ackermann-msgs ros-${ROS_DISTRO}-control-toolbox ros-${ROS_DISTRO}-nav-msgs ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-vision-msgs ros-${ROS_DISTRO}-ur-msgs ros-${ROS_DISTRO}-moveit-servo ros-${ROS_DISTRO}-moveit-visual-tools ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-pilz-industrial-motion-planner ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-ur-client-library ros-${ROS_DISTRO}-nav2-common ros-${ROS_DISTRO}-navigation2 libopencv-dev
```

### Project
You need to build and source the ROS 2 workspace first as it contains custom messages that the simulator also uses.
This workspace depends on submodules that need to be pulled first. This is done through the script (`setup_submodules.bash`) that selects a submodule's version based on the detected ROS 2 distribution.
```bash
cd ${WORKDIR}/ROSCon2023Demo/ros2_ws
./setup_submodules.bash
```
Now install all dependencies of submodules.
```bash
sudo apt install python3-colcon-common-extensions python3-vcstool python3-rosdep2
rosdep update
rosdep install --ignore-src --from-paths src/Universal_Robots_ROS2_Driver -y
```
Then build and source the workspace.
```bash
cd ${WORKDIR}/ROSCon2023Demo/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
The source command needs to be done in the same console where you build and run O3DE.

Make sure the following tools and libraries are installed on your system (they are required to build the project):
```bash
sudo apt install ninja-build libunwind-dev libxcb-xkb-dev libxcb-xfixes0-dev libxkbcommon-x11-dev libxcb-xinput-dev
```

Now, assuming that the [project's repo](https://github.com/RobotecAI/ROSCon2023Demo) was cloned to `${WORKDIR}`:
```bash
cd ${WORKDIR}/ROSCon2023Demo/Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DLY_STRIP_DEBUG_SYMBOLS=ON
cmake --build build/linux --config profile --target Editor ROSCon2023Demo.Assets ROSCon2023Demo.GameLauncher
```
You can now run the project Editor with:
```bash
cd ${WORKDIR}/ROSCon2023Demo/Project
./build/linux/bin/profile/Editor
```
### Building the release package (optional)

To build a release package with all use export script available in o3de.
Release package is a standalone package that can be run on any computer without the need to build the project, it is 
a self-contained package with all assets and binaries.
To learn more on exporting game launcher see [O3DE documentation](https://www.docs.o3de.org/docs/user-guide/packaging/project-export/project-export-pc/).

To build the game launcher and bundle assets:
```bash
cd ${WORKDIR}/o3de
./scripts/o3de.sh export-project -es ExportScripts/export_source_built_project.py --project-path ${WORKDIR}/ROSCon2023Demo/Project --seedlist ${WORKDIR}/ROSCon2023Demo/Project/AssetBundling/SeedLists/demo.seed  --fail-on-asset-errors -noserver -out ./build/release --build-tools --no-unified-launcher
```
The build package is available here:
```
${WORKDIR}/ROSCon2023Demo/Project/build/release 
└── ROSCon2023DemoGamePackage
    ├── Cache
    │   └── linux
    │       ├── engine_linux.pak
    │       └── game_linux.pak
    ├── libPhysXGpu_64.so
    ├── libVkLayer_khronos_validation.so
    ├── project.json
    ├── Registry
    │   └── IgnoreAssetProcessor.profile.setregpatch
    ├── ROSCon2023Demo.GameLauncher
    └── VkLayer_khronos_validation.json
```
Please consider copying ROS 2 workspace to the release package. The ROS 2 workspace is required to run the ROS 2 nodes, since GameLauncher does not contain dynamic libraries required by ROS 2.

```bash
# Consider copying ROS 2 workspace
mkdir -p ${WORKDIR}/ROSCon2023Demo/Project/build/release/ros2_ws/
cp -r ${WORKDIR}/ROSCon2023Demo/ros2_ws/src  ${WORKDIR}/ROSCon2023Demo/Project/build/release/ros2_ws/
```

To start a released the GameLauncher simply:
```bash
cd ${WORKDIR}/ROSCon2023Demo/ros2_ws
colcon build --symlink-install
${WORKDIR}/ROSCon2023Demo/ros2_ws/install/setup.bash
${WORKDIR}/ROSCon2023Demo/Project/build/release/ROSCon2023DemoGamePackage/ROSCon2023Demo.GameLauncher
```

This package can be moved to cloud instance or other computer.

## Running the simulation

Open the level: `DemoLevel1.prefab`.
Launch the O3DE simulation by clicking `CTRL + G` or by clicking the launch arrow next to the `Play Controls` in the top right corner.
Now go to the `ros2_ws` folder and run the all ros2 packages.
```bash
cd ${WORKDIR}/ROSCon2023Demo/ros2_ws
source install/setup.bash
ros2 launch roscon2023_demo ROSCon2023Demo.launch.py
```
In a few seconds, the robots should spawn and start moving.
For a more in-depth explanation see the [ros2_ws/README.md](ros2_ws/README.md).
> **_NOTE:_** By default, 4 robots are spawned. To change the number of robots see [#changing-robots-amount paragraph](ros2_ws/README.md#changing-robots-amount).

You can watch the simulation of a smaller warehouse `DemoLevel1.prefab` from one out of six predefined viewpoints: the first four are set towards four different loading areas and the remaining two show the wrapping station and the unloading area respectively. The viewpoints can be changed using `[1]` to `[6]` keys on the keyboard. Simulation of a large warehouse `DemoLevel2.prefab` is observed from the fly camera, which can freely move around the scene. Use `[w]`, `[s]`, `[a]`, and `[d]` keys and a mouse to change a camera position.

## Simulation of a large scene with 36 robots

### Limitations
We experienced a problem with scale and ROS 2 launch. The standard approach of a single launch file might cause the following issues:
- Some robots were not spawned.
- Some Nav2 stacks were created in a state in which they were not operational.

The problem is communication in ROS 2 which was temporarily saturated.
The number of mechanisms in ROS 2 nodes depends on the assumption that QoS for services is reliable.
It could be not true for a saturated system.
In other words, this demo is a great efficiency test for your DDS.
To counteract the impact of those limitations, we launch a system with bash scripts, and every robot uses a separate `screen` session.

### Prerequisites
1. **Two machines** connected in a 2.5 Gbps local network, ideally point-to-point.
Specification we used:
- Intel 13th Gen Core i9-13900K
- NVIDIA GeForce RTX 4080
- 64 GB of DDR4 RAM

2. Correctly set ROS 2 domain to establish the communication between two machines.
We used CycloneDDS with the following config:
```
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config
https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <Internal>
		<SocketReceiveBufferSize min="10MB" max="default" />
        </Internal>
    	<Discovery>
      		<ParticipantIndex>auto</ParticipantIndex>
      		<MaxAutoParticipantIndex>1000</MaxAutoParticipantIndex>
    	</Discovery>
    </Domain>
</CycloneDDS>
```
Please refer to [DDS tuning information](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cyclone-dds-tuning) to learn more.

### Running simulation 
1. On **Machine 1** start GameLauncher, without connecting to `AssetProcess`, with resolution of your choice (we set it to 2.5K to achieve high frame rate) and in fullscreen mode:
    ```bash
    ./ROSCon2023.GameLaucher -r_fullscreen=false -bg_ConnectToAssetProcessor=0 -r_width=2560 -r_height=1440 -r_resolutionMode=1
    ```

2. On **Machine 1**, with GameLauncher started, switch level to `DemoLevel2` by hitting `Home` key and entering command `LoadLevel demolevel2` in Debug Console.
   ![](media/level.png)

3. On **Machine 2** build ROS2 workspace (no need to build o3de project), source workspace:
   ```bash
   cd ROSCon2023Demo/ros2_ws
   colcon build --symlink-install
   source ./install/setup.bash
   ```
4. On **Machine 2** make sure you have the `screen` binary installed as it is used by bash launch scripts:
   ```bash
   sudo apt install screen
   ```
   Then start two scripts that will bring all ROS 2 software stacks:
   ```bash
    ./src/roscon2023_demo/bash/spawn.sh
    ./src/roscon2023_demo/bash/start_fleet.sh
    ```
    The `spawn.sh` script starts MoveIt2 move groups, palletization drivers and spawns all AMRs one by one.
    The second script, `start_fleet.sh`, creates multiple screen sessions to run AMR navigation.

5. To stop the demo, simply close all `screen` sessions on **Machine 2**:   
    ```bash
    killall screen
    ```

### Troubleshooting
If you intend to switch between Humble and Iron distributions, it is best to perform a clean build, or at least rebuild ROS 2 and RGL Gem.
Make sure you build the workspace and the simulation project with the same distribution (rebuild and source on change).

If your simulation does not work as intended, please first make sure that you sourced the workspace again before running the project.

Please also refer to the common [Troubleshooting Guide](https://docs.o3de.org/docs/user-guide/interactivity/robotics/troubleshooting/).

## Release notes

### ROSCon2023Demo 2.0.0 for O3DE 2409.x
Changes compared to 1.0.1:
- updated demo the newest available version of O3DE and ROS 2 Gem
- added docker support (sample Dockerfiles and additional README)
- added **DemoStereo** level showcasing HIL setup with AMD KRIA and stereo cameras (this level was used during ROSCon 2024 conference)
- moved multiple assets to standalone repositories to simplify their usage among different projects
- multiple scene optimizations
- updated README

### ROSCon2023Demo 1.0.1 for O3DE 2310.x
Changes compared to 1.0.0:
- enforced versions of dependencies (HumanWorker, OTTORobots, URRobots)
- updated UR (external) submodule
- updated README

### ROSCon2023Demo 1.0.0 for O3DE 2310.x
Initial release of the demo, prepared around ROSCon 2023 conference.

## Acknowledgments

This demo project was originally developed by [Robotec.ai](https://robotec.ai) in cooperation with [AWS Game Tech](https://aws.amazon.com/gametech/) and [AWS RoboMaker](https://aws.amazon.com/robomaker/).
