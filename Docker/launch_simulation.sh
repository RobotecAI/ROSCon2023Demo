#!/bin/bash

. /opt/ros/humble/setup.sh

pushd $ROSCON_DEMO_ROOT/ros2_ws
. ./install/setup.sh
popd 

pushd $ROSCON_SIMULATION_HOME
./ROSCon2023Demo.GameLauncher -bg_connectToAssetProcessor=0
popd

exit 0
