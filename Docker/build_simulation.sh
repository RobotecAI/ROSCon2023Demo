#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Derive the DEMO_LEVEL from ROSCON_DEMO_LEVEL (validation was done in clone_simulation.sh)
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
# Build O3DE tools for asset processing and asset bundling
###############################################################

cmake -B $ROSCON_DEMO_PROJECT/build/tools \
      -S $ROSCON_DEMO_PROJECT \
      -G "Ninja Multi-Config" \
      -DLY_DISABLE_TEST_MODULES=ON \
      -DLY_STRIP_DEBUG_SYMBOLS=ON
if [ $? -ne 0 ]
then
    echo "Error generating O3DE tools projects"
    exit 1
fi

cmake --build $ROSCON_DEMO_PROJECT/build/tools \
      --config profile \
      --target AssetProcessorBatch AssetBundlerBatch
if [ $? -ne 0 ]
then
    echo "Error building the O3DE tools projects"
    exit 1
fi

###############################################################
# Build the assets for ROSCon2023Demo
###############################################################
pushd $ROSCON_DEMO_PROJECT/build/tools/bin/profile

# Initial run to process the assets
./AssetProcessorBatch

# Secondary run to re-process ones that missed dependencies
./AssetProcessorBatch

popd

###############################################################\
# Bundle the assets for ROSCon2023Demo
###############################################################\
mkdir -p $ROSCON_DEMO_PROJECT/build/bundles
mkdir -p $ROSCON_SIMULATION_HOME/Cache/linux

# Generate the asset lists file for the game
pushd $ROSCON_DEMO_PROJECT/build/tools/bin/profile

echo "Creating the game assetList ..."
./AssetBundlerBatch assetLists \
         --assetListFile $ROSCON_DEMO_PROJECT/build/bundles/game_linux.assetList \
         --platform linux \
         --project-path $ROSCON_DEMO_PROJECT \
         --addSeed levels/$DEMO_LEVEL/$DEMO_LEVEL.spawnable \
         --allowOverwrites
if [ $? -ne 0 ]
then
    echo "Error generating asset list from levels/$DEMO_LEVEL/$DEMO_LEVEL.spawnable"
    exit 1
fi

echo "Creating the engine assetList ..."
./AssetBundlerBatch assetLists \
         --assetListFile $ROSCON_DEMO_PROJECT/build/bundles/engine_linux.assetList \
         --platform linux \
         --project-path $ROSCON_DEMO_PROJECT \
         --addDefaultSeedListFiles \
         --allowOverwrites
if [ $? -ne 0 ]
then
    echo "Error generating default engine asset list"
    exit 1
fi

echo "Creating the game asset bundle (pak) ..."
./AssetBundlerBatch bundles \
        --maxSize 2048 \
        --platform linux \
        --project-path $ROSCON_DEMO_PROJECT \
        --allowOverwrites \
        --assetListFile $ROSCON_DEMO_PROJECT/build/bundles/game_linux.assetList \
        --outputBundlePath $ROSCON_SIMULATION_HOME/Cache/linux/game_linux.pak
if [ $? -ne 0 ]
then
    echo "Error bundling generating game pak"
    exit 1
fi
             
echo "Creating the engine asset bundle (pak) ..."
./AssetBundlerBatch bundles \
        --maxSize 2048 \
        --platform linux \
        --project-path $ROSCON_DEMO_PROJECT \
        --allowOverwrites \
        --assetListFile $ROSCON_DEMO_PROJECT/build/bundles/engine_linux.assetList \
        --outputBundlePath $ROSCON_SIMULATION_HOME/Cache/linux/engine_linux.pak
if [ $? -ne 0 ]
then
    echo "Error bundling generating game pak"
    exit 1
fi

# Build the game launcher monolithically
echo "Building launcher"
cmake -B $ROSCON_DEMO_PROJECT/build/game \
      -S $ROSCON_DEMO_PROJECT \
      -G "Ninja Multi-Config" \
      -DLY_DISABLE_TEST_MODULES=ON \
      -DLY_STRIP_DEBUG_SYMBOLS=ON \
      -DLY_MONOLITHIC_GAME=ON \
      -DALLOW_SETTINGS_REGISTRY_DEVELOPMENT_OVERRIDES=0 \
    && cmake --build $ROSCON_DEMO_PROJECT/build/game \
            --config profile \
            --target ./ROSCon2023Demo.GameLauncher
if [ $? -ne 0 ]
then
    echo "Error bundling simulation launcher"
    exit 1
fi

# Package the binaries to the simulation folder
cp $ROSCON_DEMO_PROJECT/build/game/bin/profile/ROSCon2023Demo.GameLauncher $ROSCON_SIMULATION_HOME/ 
cp $ROSCON_DEMO_PROJECT/build/game/bin/profile/*.json $ROSCON_SIMULATION_HOME/ 
cp $ROSCON_DEMO_PROJECT/build/game/bin/profile/*.so $ROSCON_SIMULATION_HOME

# Cleanup
rm -rf $ROSCON_DEMO_ROOT/engine
rm -rf $ROSCON_DEMO_ROOT/gems
rm -rf $ROSCON_DEMO_ROOT/Project/build
rm -rf /root/.o3de

exit 0
