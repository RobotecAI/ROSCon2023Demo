#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

. /opt/ros/${ROS_DISTRO}/setup.sh

if [ $ROSCON_DEMO_LARGE_SCALE -eq 1 ]
then
    export CYCLONEDDS_URI=/data/workspace/roscon2023_large_cyclone_config.xml
fi

cd $ROSCON_DEMO_ROOT/ros2_ws
. ./install/setup.sh

cd $ROSCON_SIMULATION_HOME
./ROSCon2023Demo.GameLauncher -r_fullscreen=$ROCSON_DEMO_FULLSCREEN -bg_connectToAssetProcessor=0
