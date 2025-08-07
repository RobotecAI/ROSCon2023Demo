# Docker support for ROSCon2023Demo

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Any Linux distribution that supports Docker and the NVIDIA container toolkit (see below)
* At least 60 GB of free disk space
* Docker installed and configured
  * **Note** It is recommended to have Docker installed correctly and securely, so the Docker commands in this guide do not require elevated privileges (sudo) in order to run them. See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for more details.
* [NVIDIA container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
* [rocker](https://github.com/osrf/rocker)
  * **Optional** This tool simplifies injecting NVIDIA support when running Docker images. If not installed, you can still run GPU-required Docker images but with additional setup steps.
  * **Note** There known issues using rocker with Ubuntu 22.04 and NVIDIA drivers newer than version 525.

# Building the Docker image
There are 2 different Dockerscripts that build the two different types of ROSCon2023Demo images:

* **Dockerfile.O3DE** The Dockerscript used to build the Docker image that contains both the ROS 2 and O3DE simulation software and environment.
* **Dockerfile.ROS** The Dockerscript used to build the Docker image that contains only the ROS 2 relevant software and environment. It does not contain anything that is specific to the O3DE simulation.

## Building the O3DE Docker image
The script for O3DE (`Dockerfile.O3DE`) will build the ROSCon2023Demo Warehouse simulation launcher as well as the ros projects that are needed to launch the simulation. The following arguments (passed to the Docker build command with the `--build-arg` parameter) are supported to customize the Docker image.

| Argument                        | Description                                                                           | Default                                           |
| ------------------------------- | ------------------------------------------------------------------------------------- | ------------------------------------------------- |
| ROS_VERSION                     | The distro of ROS (humble or iron)                                                    | humble                                            |
| UBUNTU_VERSION                  | The supporting distro of ubuntu (focal, jammy, noble)                                 | jammy                                             |
| O3DE_REPO                       | The git repo for O3DE                                                                 | https://github.com/o3de/o3de                      |
| O3DE_BRANCH                     | The branch/tag for O3DE                                                               | 2505.1                                            |
| O3DE_COMMIT                     | The commit on the branch/tag for O3DE (or HEAD)                                       | HEAD                                              |
| O3DE_EXTRAS_REPO                | The git repo for O3DE Extras                                                          | https://github.com/o3de/o3de-extras               |
| O3DE_EXTRAS_BRANCH              | The branch/tag for O3DE Extras                                                        | 2505.1                                            |
| O3DE_EXTRAS_COMMIT              | The commit on the branch for O3DE Extras (or HEAD)                                    | HEAD                                              |
| ROSCON_DEMO_HUMAN_WORKER_REPO   | The git repo for Demo Human worker Gem                                                | https://github.com/RobotecAI/o3de-humanworker-gem |
| ROSCON_DEMO_HUMAN_WORKER_BRANCH | The branch/tag for Demo Human worker Gem                                              | 2.0.0                                             |
| ROSCON_DEMO_HUMAN_WORKER_COMMIT | The commit on the branch/tag for Demo Human worker Gem (or HEAD)                      | HEAD                                              |
| ROSCON_DEMO_UR_ROBOTS_REPO      | The git repo for Demo UR Robots Gem                                                   | https://github.com/RobotecAI/o3de-ur-robots-gem   |
| ROSCON_DEMO_UR_ROBOTS_BRANCH    | The branch/tag for Demo UR Robots Gem                                                 | 2.0.0                                             |
| ROSCON_DEMO_UR_ROBOTS_COMMIT    | The commit on the branch/tag for Demo UR Robots Gem (or HEAD)                         | HEAD                                              |
| ROSCON_DEMO_OTTO_ROBOTS_REPO    | The git repo for the Demo Otto Robots Gem                                             | https://github.com/RobotecAI/o3de-otto-robots-gem |
| ROSCON_DEMO_OTTO_ROBOTS_BRANCH  | The branch/tag for the Demo Otto Robots Gem                                           | 2.0.0                                             |
| ROSCON_DEMO_OTTO_ROBOTS_COMMIT  | The commit on the branch/tag for the Demo Otto Robots Gem (or HEAD)                   | HEAD                                              |
| ROSCON_DEMO_REPO                | The git repo for ROSCon2023 Warehouse Demo                                            | https://github.com/RobotecAI/ROSCon2023Demo.git   |
| ROSCON_DEMO_BRANCH              | The branch/tag for ROSCon2023 Warehouse Demo                                          | 2.0.0                                             |
| ROSCON_DEMO_COMMIT              | The commit on the branch/tag for ROSCon2023 Warehouse Demo (or HEAD)                  | HEAD                                              |
| ROCSON_DEMO_LEVEL               | The startup level (level1 or level2). **See Notes below**                             | level1                                            |
| ROCSON_DEMO_FULLSCREEN          | Option to launch the simulation in fullscreen mode (0=no, 1=yes)                      | 0                                                 |
| ROSCON_DEMO_LARGE_SCALE         | Option to enable large scale simulation (0=no, 1=yes) (see [README.md]()../README.md) | 0                                                 |

To build the Docker image using the default values, use the following command

```
docker build -f Dockerfile.O3DE -t roscon2023_demo/o3de:latest .
```

If you want to pull a different variant of the image based on a different fork, you can run a command similar to the following

```
docker build -f Dockerfile.O3DE --build-arg ROSCON_DEMO_REPO=https://github.com/myfork/ROSCon2023Demo.git --build-arg ROCSON_DEMO_LEVEL=level2 -t myfork/roscon2023_demo/o3de:latest .
```

> **Note:**: `Dockerfile.O3DE` clones multiple Gems that are not listed in the table above. These are assets only Gems or tooling Gems. The working commit hashes are either hard-coded in the scripts or the newest versions are pulled from the repository (for backward compatible Gems).

## Building the ROS Docker image
The O3DE Docker image can reach 10 GB in size, so if you want to create a separate image with just the ROS related projects, then you would use the reduced script for just ROS (`Dockerfile.ROS`). The script will also accept a reduced number of arguments if you need to customize where to synchronize the ROSCon2023 project from.

| Argument                | Description                                                                           | Default                                         |
| ----------------------- | ------------------------------------------------------------------------------------- | ----------------------------------------------- |
| ROS_VERSION             | The distro of ROS (humble, or iron)                                                   | humble                                          |
| UBUNTU_VERSION          | The supporting distro of Ubuntu (focal, jammy, noble)                                 | jammy                                           |
| ROSCON_DEMO_REPO        | The git repo for ROSCon2023 Warehouse Demo                                            | https://github.com/RobotecAI/ROSCon2023Demo.git |
| ROSCON_DEMO_BRANCH      | The branch/tag for ROSCon2023 Warehouse Demo                                          | 2.0.0                                           |
| ROSCON_DEMO_COMMIT      | The commit on the branch/tag for ROSCon2023 Warehouse Demo (or HEAD)                  | HEAD                                            |
| ROSCON_DEMO_LARGE_SCALE | Option to enable large scale simulation (0=no, 1=yes) (see [README.md]()../README.md) | 0                                               |

To build the Docker image using the default values, use the following command

```
docker build -f Dockerfile.ROS -t roscon2023_demo/ros:latest .
```

> **Note:** By default, the O3DE simulation launcher is built to run the smaller warehouse scene. To create a Docker image with the large warehouse, the `ROSCON_DEMO_LEVEL` argument needs to be set to `level2`:
>
>```
>docker build -f Dockerfile.O3DE --build-arg ROCSON_DEMO_LEVEL=level2 -t roscon2023_demo/o3de:latest .
>```

# Running the Docker image
The Docker image for the simulation (O3DE) requires Vulkan and GPU acceleration provided by the NVIDIA drivers and container toolkit. The following directions will describe how to launch the Docker containers, utilizing the host Linux machine's X11 display and NVIDIA drivers, and connecting to the default 'bridge' network. (For advanced network isolation, refer to Docker's command-line reference for [network](https://docs.docker.com/reference/cli/docker/container/run/#network))

```
xhost +local:root
docker run --rm --gpus all -e DISPLAY=:1 --network="bridge" -v /tmp/.X11-unix:/tmp/.X11-unix -it roscon2023_demo/o3de /bin/bash
```
or by using [rocker](https://github.com/osrf/rocker):

```
rocker --x11 --nvidia --network="bridge" roscon2023_demo/o3de /bin/bash
```

The image provides a few convenience scripts which are accessible once you are logged into the Docker container.

| File                   | Description                                                                                           |
| ---------------------- | ----------------------------------------------------------------------------------------------------- |
| `launch_ros.sh`        | Convenience script to launch the ROS 2 stack                                                          |
| `launch_simulation.sh` | Convenience script to launch the Warehouse simulation                                                 |
| `launch_ros_fleet.sh`  | (Dockerfile.ROS only) Convenience script to launch the large scale fleet of robots                    |
| `git_commit.txt`       | Summary of the git source repositories, their branches and commits that were used to build this image |


While logged into a O3DE Docker container, the scripts provide simplified way to launch the ROS 2 stack and the Warehouse simulation client with proper environment setup. However, each of these processes output a large amount of status logs, and unless they are run as a background process, they will take over the current shell. It is recommended that you both redirect the output of the scripts to a specific log file, and run them as a background process.

First, launch the O3DE warehouse simulation client in the background with its log directed to `/data/workspace/simulation.log` :

```
./launch_simulation.sh > simulation.log &>1 &
```

> **Note:** The client may hide the cursor and then take the window focus. You may need to switch back to the Docker container window by using the `ALT+TAB` key combination to get back to the console.

Next, start the ROS 2 stack in the background and direct the log to `/data/workspace/ros2.log` : 

```
./launch_ros.sh > /data/workspace/ros2.log &>1 &
```

Depending on which level is specified with the `ROCSON_DEMO_LEVEL` the simulation will start with the configured warehouse scene. The ROS 2 stack will spawn the worker robots and start their navigation tasks.

# Large scale simulation

As described in the main [README.md](../README.md), running a large scale simulation with 36 robots is processor and resource intensive. It is recommended to run the ROS 2 stack on a different machine. The Docker image for ROS provides just the ROS 2 components needed to start ROS and spawning and starting the navigation stacks. For the large scale simulation, you will also need to build a Docker image for `level2` which contains the large warehouse (the default `level1` was not designed for the large scale simulation).

## Building the large scale Docker images

You will need to build the O3DE and ROS Docker images on separate machines. The machine that runs the simulation will require a higher-end machine spec to handle the heavy graphics load. Refer to the main [README.md](../README.md) for more information.

On the machine meant for the simulation, run the following command to build the O3DE Docker image for the large scale simulation:
```
docker build -f Dockerfile.O3DE --build-arg ROCSON_DEMO_LEVEL=level2 --build-arg ROSCON_DEMO_LARGE_SCALE=1 -t roscon2023_demo_large/o3de:latest .
```
This will create the docker image `roscon2023_demo_large/o3de:latest` that will launch the Docker container from.

On the machine meant for the ROS stack to spawn and launch the 36 robots into the simulation, run the following command:
```
docker build -f Dockerfile.ROS --build-arg ROSCON_DEMO_LARGE_SCALE=1 -t roscon2023_demo_large/ros:latest .
```
This will create the docker image `roscon2023_demo_large/ros:latest` that will launch the Docker container from.

Also make sure that the two machines are on the same subnet.

## Running the large scale Docker images

> **Note:** The cyclone DDS Tuning described in [README.md](../README.md) is mostly handled in the ROS Docker image, but a critical part of the [tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html#cyclone-dds-tuning) is to increase the maximum buffer receive size `rmem_max`. This is a kernel setting and the container shares the value that the host is set to. 

On the machine that will run the simulation, run the following commands to start the Docker image and log into the container:

```
xhost +local:root
sudo sysctl -w net.core.rmem_max=2147483647
docker run --rm --gpus all -e DISPLAY=:1 --network="bridge" -v /tmp/.X11-unix:/tmp/.X11-unix -it roscon2023_demo_large/o3de /bin/bash
```
or by using [rocker](https://github.com/osrf/rocker):
```
sudo sysctl -w net.core.rmem_max=2147483647
rocker --x11 --nvidia --network="bridge" roscon2023_demo_large/o3de /bin/bash
```

When logged into the Docker container, launch the O3DE warehouse simulation client in the background with its log directed to `/data/workspace/simulation.log` :

```
./launch_simulation.sh > simulation.log &>1 &
```

On the machine that will spawn and run the ROS robot navigation stack, run the following commands to start the Docker image and log into the container:
```
xhost +local:root
sudo sysctl -w net.core.rmem_max=2147483647
docker run --rm --gpus all -e DISPLAY=:1 --network="bridge" -v /tmp/.X11-unix:/tmp/.X11-unix -it roscon2023_demo_large/ros /bin/bash
```
or by using [rocker](https://github.com/osrf/rocker):
```
sudo sysctl -w net.core.rmem_max=2147483647
rocker --x11 --nvidia --network="bridge" roscon2023_demo_large/ros /bin/bash
```

Next, start the script to spawn and run the fleet:

```
./launch_ros_fleet.sh
```
You can shut down the simulation shutting down the Docker containers. For more information on running the simulation, refer to the [README.md](../README.md) file located in the root directory.
