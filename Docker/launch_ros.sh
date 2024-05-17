#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

. /opt/ros/${ROS_DISTRO}/setup.sh

cd $ROSCON_DEMO_ROOT/ros2_ws
. ./install/setup.sh

cd $ROSCON_DEMO_ROOT/ros2_ws
ros2 launch roscon2023_demo ROSCon2023Demo.launch.py

exit 0
