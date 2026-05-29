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

echo "Initializing submodules (excluding engine/o3de and engine/o3de-extras)"
git -C $ROSCON_DEMO_ROOT submodule update --init -- \
    gems/robotec-o3de-tools \
    gems/robotec-warehouse-assets \
    gems/robotec-generic-assets \
    gems/o3de-humanworker-gem \
    gems/o3de-ur-robots-gem \
    gems/o3de-otto-robots-gem
if [ $? -ne 0 ]
then
    echo "Error initializing submodules"
    exit 1
fi

$O3DE_INSTALL_DIR/scripts/o3de.sh register -pp $ROSCON_DEMO_PROJECT
if [ $? -ne 0 ]
then
    echo "Error registering the ROSCon2023Demo project"
    exit 1
fi

###############################################################
# Set the filename of the level in the registry file
###############################################################
sed -i "s/demolevel1/${DEMO_LEVEL}/g" $ROSCON_DEMO_PROJECT/Registry/load_level.setreg

exit 0
