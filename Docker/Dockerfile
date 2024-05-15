# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

ARG ROS_VERSION=humble
ARG UBUNTU_VERSION=jammy

FROM ros:${ROS_VERSION}-ros-base-${UBUNTU_VERSION}

ENV WORKSPACE=/data/workspace

WORKDIR $WORKSPACE

# O3DE Arguments
ARG O3DE_REPO=https://github.com/o3de/o3de.git
ARG O3DE_BRANCH=development
ARG O3DE_COMMIT=HEAD

# O3DE Extras Arguments
ARG O3DE_EXTRAS_REPO=https://github.com/o3de/o3de-extras.git
ARG O3DE_EXTRAS_BRANCH=development
ARG O3DE_EXTRAS_COMMIT=HEAD

# RosCon2023 Human Worker Arguments
ARG ROSCON_DEMO_HUMAN_WORKER_REPO=https://github.com/RobotecAI/o3de-humanworker-gem
ARG ROSCON_DEMO_HUMAN_WORKER_BRANCH=development
ARG ROSCON_DEMO_HUMAN_WORKER_COMMIT=HEAD

# RosCon2023 UR10 and UR20 Robots Gem Arguments
ARG ROSCON_DEMO_UR_ROBOTS_REPO=https://github.com/RobotecAI/o3de-ur-robots-gem
ARG ROSCON_DEMO_UR_ROBOTS_BRANCH=development
ARG ROSCON_DEMO_UR_ROBOTS_COMMIT=HEAD

# RosCon2023 OTTO 600 and OTTO 1500 Robots Gem Arguments
ARG ROSCON_DEMO_OTTO_ROBOTS_REPO=https://github.com/RobotecAI/o3de-otto-robots-gem
ARG ROSCON_DEMO_OTTO_ROBOTS_BRANCH=development
ARG ROSCON_DEMO_OTTO_ROBOTS_COMMIT=HEAD

# ROSConDemo Arguments
ARG ROSCON_DEMO_REPO=https://github.com/RobotecAI/ROSCon2023Demo.git
ARG ROSCON_DEMO_BRANCH=development
ARG ROSCON_DEMO_COMMIT=HEAD

# O3DE Environment
ENV O3DE_REPO=$O3DE_REPO
ENV O3DE_BRANCH=$O3DE_BRANCH
ENV O3DE_COMMIT=$O3DE_COMMIT
ENV O3DE_ROOT=$WORKSPACE/o3de

# O3DE Extras Environment Variables
ENV O3DE_EXTRAS_REPO=$O3DE_EXTRAS_REPO
ENV O3DE_EXTRAS_BRANCH=$O3DE_EXTRAS_BRANCH
ENV O3DE_EXTRAS_COMMIT=$O3DE_EXTRAS_COMMIT
ENV O3DE_EXTRAS_ROOT=$WORKSPACE/o3de-extras

# RosCon2023 Human Worker Environment Variables
ENV ROSCON_DEMO_HUMAN_WORKER_REPO=$ROSCON_DEMO_HUMAN_WORKER_REPO
ENV ROSCON_DEMO_HUMAN_WORKER_BRANCH=$ROSCON_DEMO_HUMAN_WORKER_BRANCH
ENV ROSCON_DEMO_HUMAN_WORKER_COMMIT=$ROSCON_DEMO_HUMAN_WORKER_COMMIT
ENV ROSCON_DEMO_HUMAN_WORKER_ROOT=$WORKSPACE/o3de-humanworker-gem

# RosCon2023 UR10 and UR20 Robots Gem Environment Variables
ENV ROSCON_DEMO_UR_ROBOTS_REPO=$ROSCON_DEMO_UR_ROBOTS_REPO
ENV ROSCON_DEMO_UR_ROBOTS_BRANCH=$ROSCON_DEMO_UR_ROBOTS_BRANCH
ENV ROSCON_DEMO_UR_ROBOTS_COMMIT=$ROSCON_DEMO_UR_ROBOTS_COMMIT
ENV ROSCON_DEMO_UR_ROBOTS_ROOT=$WORKSPACE/o3de-ur-robots-gem

# RosCon2023 OTTO 600 and OTTO 1500 Robots Gem Environment Variables
ENV ROSCON_DEMO_OTTO_ROBOTS_REPO=$ROSCON_DEMO_OTTO_ROBOTS_REPO
ENV ROSCON_DEMO_OTTO_ROBOTS_BRANCH=$ROSCON_DEMO_OTTO_ROBOTS_BRANCH
ENV ROSCON_DEMO_OTTO_ROBOTS_COMMIT=$ROSCON_DEMO_OTTO_ROBOTS_COMMIT
ENV ROSCON_DEMO_OTTO_ROBOTS_ROOT=$WORKSPACE/o3de-otto-robots-gem

# ROSConDemo Environment Variables
ENV ROSCON_DEMO_REPO=$ROSCON_DEMO_REPO
ENV ROSCON_DEMO_BRANCH=$ROSCON_DEMO_BRANCH
ENV ROSCON_DEMO_COMMIT=$ROSCON_DEMO_COMMIT

ENV ROSCON_DEMO_ROOT=$WORKSPACE/ROSCon2023Demo
ENV ROSCON_DEMO_PROJECT=$ROSCON_DEMO_ROOT/Project
ENV ROSCON_DEMO_NAV_ROOT=$WORKSPACE/ros2_nav
ENV ROSCON_DEMO_SEEDLIST=$ROSCON_DEMO_ROOT/Project/AssetBundling/SeedLists/DemoLevel1.seed

ENV ROSCON_SIMULATION_HOME=$WORKSPACE/Simulation

ENV LANG=en_US.UTF-8

# Setup time zone and locale data (necessary for SSL and HTTPS packages)
RUN apt-get update \
    && DEBIAN_FRONTEND="noninteractive" apt-get -y install tzdata locales keyboard-configuration curl \
    && sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen \
    && dpkg-reconfigure --frontend=noninteractive locales \
    && update-locale LANG=en_US.UTF-8 \
    && sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && rm -rf /var/lib/apt/lists/*

# Install the required ubuntu packages
RUN apt-get update \
    && apt-get install -y \
            binutils \
            clang \
            cmake \
            git \
            git-lfs \
            libglu1-mesa-dev \
            libxcb-xinerama0 \
            libfontconfig1-dev \
            libnvidia-gl-470 \
            libxcb-xkb-dev \
            libxkbcommon-x11-dev \
            libxkbcommon-dev \
            libxcb-xfixes0-dev \
            libxcb-xinput-dev \
            libxcb-xinput0 \
            libpcre2-16-0 \
            libunwind-dev \
            ninja-build \
            python3-pip \
            software-properties-common \
            ros-${ROS_DISTRO}-ackermann-msgs \
            ros-${ROS_DISTRO}-control-toolbox \
            ros-${ROS_DISTRO}-nav-msgs \
            ros-${ROS_DISTRO}-gazebo-msgs \
            ros-${ROS_DISTRO}-xacro \
            ros-${ROS_DISTRO}-vision-msgs \
            ros-${ROS_DISTRO}-nav2-msgs \
            ros-${ROS_DISTRO}-ur-msgs \
            ros-${ROS_DISTRO}-moveit-servo \
            ros-${ROS_DISTRO}-moveit-visual-tools \
            ros-${ROS_DISTRO}-moveit \
            ros-${ROS_DISTRO}-pilz-industrial-motion-planner \
            ros-${ROS_DISTRO}-cyclonedds \
            ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
            ros-${ROS_DISTRO}-controller-manager \
            ros-${ROS_DISTRO}-ur-client-library \
            ros-${ROS_DISTRO}-nav2-common \
            ros-${ROS_DISTRO}-navigation2 \
            ros-${ROS_DISTRO}-ur-description \
            ros-${ROS_DISTRO}-force-torque-sensor-broadcaster \
            python3-vcstool \
            python3-rosdep2 \
            python3-colcon-common-extensions \
    && pip install python-statemachine

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY roscon2023.patch $WORKSPACE/
COPY RosConDemoSeedList.seed $WORKSPACE/
COPY build_simulation.sh $WORKSPACE/
COPY game.cfg $ROSCON_SIMULATION_HOME/
COPY launch_simulation.sh /data/workspace/

RUN cd $WORKSPACE \
    && ./build_simulation.sh

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENTRYPOINT ["/bin/bash", "-c"]
