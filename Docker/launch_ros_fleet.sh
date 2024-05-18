#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

. /opt/ros/${ROS_DISTRO}/setup.sh

if [ $ROSCON_DEMO_LARGE_SCALE -eq 0 ]
then
    echo "The ROSCON_DEMO_LARGE_SCALE argument was not set to 1 on this image."
    exit 1
else
    export CYCLONEDDS_URI=/data/workspace/roscon2023_large_cyclone_config.xml
fi

cd $ROSCON_DEMO_ROOT/ros2_ws/src/roscon2023_demo/bash/

. ./spawn.sh

. ./start_fleet.sh

exit 0
