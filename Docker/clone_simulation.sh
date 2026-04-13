#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Validate the ROSCON_DEMO_LEVEL value before starting the longer git processes
if [ "$ROSCON_DEMO_LEVEL" == 'level1' ]
then
    DEMO_LEVEL=demolevel1
elif [ "$ROSCON_DEMO_LEVEL" == 'level2' ]
then
    DEMO_LEVEL=demolevel2
else
    echo "Invalid 'level' argument '$ROSCON_DEMO_LEVEL'. Must be level1 or level2."
    exit 1
fi

# Initialize ROS
. /opt/ros/${ROS_DISTRO}/setup.sh


###############################################################
# Clone and register the ROSCon2023Demo
###############################################################
git config --global http.postBuffer 524288000
git config --global http.lowSpeedLimit 0
git config --global http.lowSpeedTime 300

echo "Cloning the RosCon2023Demo project"
if [ "$ROSCON_DEMO_COMMIT" == "HEAD" ]
then
    git clone --single-branch --depth 1 -b $ROSCON_DEMO_BRANCH $ROSCON_DEMO_REPO $ROSCON_DEMO_ROOT && \
        git -C $ROSCON_DEMO_ROOT lfs install && \
        git -C $ROSCON_DEMO_ROOT lfs pull
else
    git clone --single-branch -b $ROSCON_DEMO_BRANCH $ROSCON_DEMO_REPO $ROSCON_DEMO_ROOT && \
        git -C $ROSCON_DEMO_ROOT lfs install && \
        git -C $ROSCON_DEMO_ROOT lfs pull && \
        git -C $ROSCON_DEMO_ROOT reset --hard $ROSCON_DEMO_COMMIT
fi
if [ $? -ne 0 ]
then
    echo "Error cloning RosCon2023Demo project $ROSCON_DEMO_REPO"
    exit 1
fi

echo "Initializing submodules"
git -C $ROSCON_DEMO_ROOT submodule update --init --recursive
if [ $? -ne 0 ]
then
    echo "Error initializing submodules"
    exit 1
fi

$ROSCON_DEMO_ROOT/engine/o3de/python/get_python.sh
if [ $? -ne 0 ]
then
    echo "Error downloading/configuring O3DE Python"
    exit 1
fi

$ROSCON_DEMO_ROOT/engine/o3de/scripts/o3de.sh register -ep $ROSCON_DEMO_ROOT/engine/o3de
if [ $? -ne 0 ]
then
    echo "Error registering the O3DE engine"
    exit 1
fi

###############################################################
# Set the filename of the level in the registry file
###############################################################
sed -i "s/demolevel1/${DEMO_LEVEL}/g" $ROSCON_DEMO_PROJECT/Registry/load_level.setreg

exit 0
