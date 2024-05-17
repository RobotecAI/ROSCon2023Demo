# Docker support for ROSCon2023Demo

## Prerequisites

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Any Linux distribution that supports Docker and the Nvidia container toolkey (see below)
* At least 60 GB of free disk space
* Docker installed and configured
  * **Note** It is recommended to have Docker installed correctly and in a secure manner so that the Docker commands in this guide do not require elevated priviledges (sudo) in order to run them. See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for more details.
* [NVidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
* [rocker](https://github.com/osrf/rocker)
  * **Optional** This tool simplifies injecting nvidia support when running Docker images. If not installed, you can still run GPU-required Docker images but with additional setup steps.

# Building the Docker image
There are 2 different Dockerscripts that build the two different types of ROSCon2023Demo images:

* **Dockerfile.ROS** The Dockerscript used to build the Docker image that contains only the ROS2 relevant software and environment. It does not contain anything that is specific to the O3DE simulation.
* **Dockerfile.O3DE** The Dockerscript used to build the Docker image that contains both the ROS2 and O3DE simulation software and environment.

## Building the O3DE Docker image
The script for O3DE (Dockerfile.O3DE) will build the ROSCon2023Demo Warehouse simulation launcher as well as the ros projects that needed to launch the simulation. The following arguments (passed to the Docker build command with the `--build-arg` parameter) are supported to customize the Docker image.

| Argument                        | Description                                                      | Default     
|---------------------------------|------------------------------------------------------------------|-------------
| ROS_VERSION                     | The distro of ROS (galactic, humble, or iron)                    | humble      
| UBUNTU_VERSION                  | The supporting distro of ubuntu (focal, jammy, noble)            | jammy
| O3DE_REPO                       | The git repo for O3DE                                            | https://github.com/o3de/o3de.git
| O3DE_BRANCH                     | The branch for O3DE                                              | development
| O3DE_COMMIT                     | The commit on the branch for O3DE (or HEAD)                      | HEAD
| O3DE_EXTRAS_REPO                | The git repo for O3DE Extras                                     | https://github.com/o3de/o3de-extras.git
| O3DE_EXTRAS_BRANCH              | The branch for O3DE Extras                                       | development
| O3DE_EXTRAS_COMMIT              | The commit on the branch for O3DE Extras (or HEAD)               | HEAD
| ROSCON_DEMO_HUMAN_WORKER_REPO   | The git repo for Demo Human worker Gem                           | https://github.com/RobotecAI/o3de-humanworker-gem
| ROSCON_DEMO_HUMAN_WORKER_BRANCH | The branch for Demo Human worker Gem                             | development
| ROSCON_DEMO_HUMAN_WORKER_COMMIT | The commit on the branch for Demo Human worker Gem (or HEAD)     | HEAD
| ROSCON_DEMO_UR_ROBOTS_REPO      | The git repo for Demo UR Robots Gem                              | https://github.com/RobotecAI/o3de-ur-robots-gem
| ROSCON_DEMO_UR_ROBOTS_BRANCH    | The branch for Demo UR Robots Gem                                | development
| ROSCON_DEMO_UR_ROBOTS_COMMIT    | The commit on the branch for Demo UR Robots Gem (or HEAD)        | HEAD
| ROSCON_DEMO_OTTO_ROBOTS_REPO    | The git repo for the Demo Otto Robots Gem                        | https://github.com/RobotecAI/o3de-otto-robots-gem
| ROSCON_DEMO_OTTO_ROBOTS_BRANCH  | The branch for the Demo Otto Robots Gem                          | development
| ROSCON_DEMO_OTTO_ROBOTS_COMMIT  | The commit on the branch for the Demo Otto Robots Gem (or HEAD)  | HEAD
| ROSCON_DEMO_REPO                | The git repo for ROSCon2023 Warehouse Demo                       | https://github.com/RobotecAI/ROSCon2023Demo.git
| ROSCON_DEMO_BRANCH              | The branch for ROSCon2023 Warehouse Demo                         | development
| ROSCON_DEMO_COMMIT              | The commit on the branch for ROSCon2023 Warehouse Demo (or HEAD) | HEAD
| ROCSON_DEMO_LEVEL               | The startup level (level1 or level2). **See Notes below**        | level1

To build the Docker image using the default values, use the following command

```
docker build -f Dockerfile.O3DE -t roscon2023_demo/o3de:latest .
```

If you want to pull a different variant of the image based on a different fork, you can run a command similar to the following

```
docker build -f Dockerfile.O3DE --build-arg ROSCON_DEMO_REPO=https://github.com/myfork/ROSCon2023Demo.git --build-arg ROCSON_DEMO_LEVEL=level2 -t myfork/roscon2023_demo/o3de:latest .
```

## Building the ROS Docker image
The O3DE Docker image can reach 10 GB in size, so if you want to create a separate image with just the ROS related projects, then you would use the reduced script for just ROS (Dockerfile.ROS). The script will also accept a reduced number of arguments if you need to customize where to synchronize the ROSCon2023 project from.

| Argument                        | Description                                                      | Default     
|---------------------------------|------------------------------------------------------------------|-------------
| ROS_VERSION                     | The distro of ROS (galactic, humble, or iron)                    | humble      
| UBUNTU_VERSION                  | The supporting distro of ubuntu (focal, jammy, noble)            | jammy
| ROSCON_DEMO_REPO                | The git repo for ROSCon2023 Warehouse Demo                       | https://github.com/RobotecAI/ROSCon2023Demo.git
| ROSCON_DEMO_BRANCH              | The branch for ROSCon2023 Warehouse Demo                         | development
| ROSCON_DEMO_COMMIT              | The commit on the branch for ROSCon2023 Warehouse Demo (or HEAD) | HEAD

To build the Docker image using the default values, use the following command

```
docker build -f Dockerfile.ROS -t roscon2023_demo/ros:latest .
```

# Running the Docker image(s)
The Docker image for the simulation (O3DE) requires Vulkan and GPU acceleration provided by the NVIDIA drivers and container toolkit. The following directions will describe how to launch the Docker containers, utilizing the host Linux machine's X11 display and nvidia drivers, and connecting to the default 'bridge' network. (For advanced network isolation, refer to Docker's command-line reference for [network](https://docs.docker.com/reference/cli/docker/container/run/#network))
