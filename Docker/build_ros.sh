#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#


# Initialize ROS
. /opt/ros/${ROS_DISTRO}/setup.sh

############################################################### \
# Clone and register the ROSCon2023Demo \
###############################################################
echo "Cloning the ROSCon2023Demo project"
git clone --single-branch -b $ROSCON_DEMO_BRANCH $ROSCON_DEMO_REPO $ROSCON_DEMO_ROOT && \
    git -C $ROSCON_DEMO_ROOT lfs install && \
    git -C $ROSCON_DEMO_ROOT lfs pull && \
    git -C $ROSCON_DEMO_ROOT reset --hard $ROSCON_DEMO_COMMIT 
if [ $? -ne 0 ]
then
    echo "Error cloning ROSCon2023Demo project $ROSCON_DEMO_REPO"
    exit 1
fi

###############################################################################
# Track the git commits from all the repos
###############################################################################
echo -e "\n\
Repository                        | Commit    | Branch\n\
----------------------------------+-----------------------------------------\n\
ROSCon2023Demo Project            | $(git -C $ROSCON_DEMO_ROOT rev-parse --short HEAD)   | $ROSCON_DEMO_BRANCH\n\
\n\
" >> $WORKSPACE/git_commit.txt


###############################################################
# Build and install the additional ROS 2 packages and drivers
###############################################################
pushd $ROSCON_DEMO_ROOT/ros2_ws && \
      ./setup_submodules.bash && \
      rosdep update && \
      rosdep install --ignore-src --from-paths src/Universal_Robots_ROS2_Driver -y && \
      colcon build --symlink-install && \
      . ./install/setup.sh && \
      popd

if [ $? -ne 0 ]
then
    echo "Error building and installing additional ROS 2 packages and drivers"
    exit 1
fi

# Cleanup
rm -rf $ROSCON_DEMO_ROOT/Project

exit 0
