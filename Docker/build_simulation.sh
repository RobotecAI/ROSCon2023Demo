#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Initialize ROS
. /opt/ros/${ROS_DISTRO}/setup.sh

###############################################################
# Build and install the additional ROS 2 packages and drivers
###############################################################
pushd $ROSCON_DEMO_ROOT/ros2_ws && \
      rosdep update && \
      colcon build --symlink-install && \
      . ./install/setup.sh && \
      popd

if [ $? -ne 0 ]
then
    echo "Error building and installing additional ROS 2 packages and drivers"
    exit 1
fi

###############################################################
# Build assets and game launcher
###############################################################
cmake -B $ROSCON_DEMO_PROJECT/build/linux \
      -S $ROSCON_DEMO_PROJECT \
      -G "Ninja Multi-Config" \
      -DLY_DISABLE_TEST_MODULES=ON \
      -DLY_STRIP_DEBUG_SYMBOLS=ON
if [ $? -ne 0 ]
then
    echo "Error configuring cmake"
    exit 1
fi

cmake --build $ROSCON_DEMO_PROJECT/build/linux \
      --config profile \
      --target ROSCon2023Demo ROSCon2023Demo.Assets ROSCon2023Demo.GameLauncher
if [ $? -ne 0 ]
then
    echo "Error building assets and launcher"
    exit 1
fi

###############################################################
# Bundle assets and package the simulation
###############################################################
python3 -m pip install --break-system-packages requests puremagic psutil packaging resolvelib

$O3DE_INSTALL_DIR/scripts/o3de.sh export-project \
    -es ExportScripts/export_source_built_project.py \
    --project-path $ROSCON_DEMO_PROJECT \
    --seedlist $ROSCON_DEMO_PROJECT/AssetBundling/SeedLists/demo.seed \
    --fail-on-asset-errors \
    --launcher-build-path $ROSCON_DEMO_PROJECT/build/linux \
    -noserver \
    --no-unified-launcher \
    -out $ROSCON_DEMO_PROJECT/build/release
if [ $? -ne 0 ]
then
    echo "Error exporting the simulation project"
    exit 1
fi

###############################################################
# Move the package to the simulation home and remove build artifacts
###############################################################
mkdir -p $ROSCON_SIMULATION_HOME
cp -r $ROSCON_DEMO_PROJECT/build/release/ROSCon2023DemoGamePackage/. $ROSCON_SIMULATION_HOME/

rm -rf $ROSCON_DEMO_ROOT/gems
rm -rf $ROSCON_DEMO_PROJECT/build
rm -rf $ROSCON_DEMO_PROJECT/Cache
rm -rf /root/.o3de

exit 0
