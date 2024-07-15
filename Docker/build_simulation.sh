#!/bin/bash
#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#

# Validate the ROCSON_DEMO_LEVEL value before starting the longer git processes
if [ "$ROCSON_DEMO_LEVEL" == 'level1' ]
then
    DEMO_LEVEL=demolevel1
elif [ "$ROCSON_DEMO_LEVEL" == 'level2' ]
then    
    DEMO_LEVEL=demolevel2
else
    echo "Invalid 'level' argument '$ROCSON_DEMO_LEVEL'. Must be level1 or level2."
    exit 1
fi

# Initialize ROS
. /opt/ros/${ROS_DISTRO}/setup.sh

###############################################################
# Clone and bootstrap O3DE
###############################################################
echo "Cloning o3de"
git clone --single-branch -b $O3DE_BRANCH $O3DE_REPO $O3DE_ROOT && \
    git -C $O3DE_ROOT lfs install && \
    git -C $O3DE_ROOT lfs pull && \
    git -C $O3DE_ROOT reset --hard $O3DE_COMMIT 
if [ $? -ne 0 ]
then
    echo "Error cloning o3de from $O3DE_REPO"
    exit 1
fi

$WORKSPACE/o3de/python/get_python.sh && \
    $WORKSPACE/o3de/scripts/o3de.sh register -ep $O3DE_ROOT
if [ $? -ne 0 ]
then
    echo "Error bootstrapping O3DE from $O3DE_REPO"
    exit 1
fi

  
###############################################################
# Clone and register o3de-extras
###############################################################
echo "Cloning o3de-extras"
git clone --single-branch -b $O3DE_EXTRAS_BRANCH $O3DE_EXTRAS_REPO $O3DE_EXTRAS_ROOT && \
    git -C $O3DE_EXTRAS_ROOT lfs install && \
    git -C $O3DE_EXTRAS_ROOT lfs pull && \
    git -C $O3DE_EXTRAS_ROOT reset --hard $O3DE_EXTRAS_COMMIT
if [ $? -ne 0 ]
then
    echo "Error cloning o3de-extras from $O3DE_EXTRAS_REPO"
    exit 1
fi

$WORKSPACE/o3de/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2 && \
    $WORKSPACE/o3de/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ProteusRobot && \
    $WORKSPACE/o3de/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/WarehouseAssets && \
    $WORKSPACE/o3de/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/WarehouseAutomation
if [ $? -ne 0 ]
then
    echo "Error registering o3de-extras gems"
    exit 1
fi


############################################################### 
# Clone and register the ROSCON Demo Human Worker Gem
###############################################################
echo "Cloning the RosCon2023 Demo Human Worker Gem"
git clone --single-branch -b $ROSCON_DEMO_HUMAN_WORKER_BRANCH $ROSCON_DEMO_HUMAN_WORKER_REPO $ROSCON_DEMO_HUMAN_WORKER_ROOT && \
    git -C $ROSCON_DEMO_HUMAN_WORKER_ROOT lfs install && \
    git -C $ROSCON_DEMO_HUMAN_WORKER_ROOT lfs pull && \
    git -C $ROSCON_DEMO_HUMAN_WORKER_ROOT reset --hard $ROSCON_DEMO_HUMAN_WORKER_COMMIT 
if [ $? -ne 0 ]
then
    echo "Error cloning RosCon2023 Demo Human Worker Gem from $ROSCON_DEMO_HUMAN_WORKER_REPO"
    exit 1
fi

$WORKSPACE/o3de/scripts/o3de.sh register -gp $ROSCON_DEMO_HUMAN_WORKER_ROOT
if [ $? -ne 0 ]
then
    echo "Error registering the RosCon2023 Demo Human Worker Gem"
    exit 1
fi


###############################################################
# Clone and register the RosCon2023 UR10 and UR20 Robots Gem
############################################################### \
echo "Cloning RosCon2023 UR10 and UR20 Robots Gem"
git clone --single-branch -b $ROSCON_DEMO_UR_ROBOTS_BRANCH $ROSCON_DEMO_UR_ROBOTS_REPO $ROSCON_DEMO_UR_ROBOTS_ROOT && \
    git -C $ROSCON_DEMO_UR_ROBOTS_ROOT lfs install && \
    git -C $ROSCON_DEMO_UR_ROBOTS_ROOT lfs pull && \
    git -C $ROSCON_DEMO_UR_ROBOTS_ROOT reset --hard $ROSCON_DEMO_UR_ROBOTS_COMMIT 
if [ $? -ne 0 ]
then
    echo "Error cloning RosCon2023 UR10 and UR20 Robots Gem from $ROSCON_DEMO_UR_ROBOTS_REPO"
    exit 1
fi

$WORKSPACE/o3de/scripts/o3de.sh register -gp $ROSCON_DEMO_UR_ROBOTS_ROOT
if [ $? -ne 0 ]
then
    echo "Error registering the RosCon2023 UR10 and UR20 Robots Gem"
    exit 1
fi


#######################################################################
# Clone and register the RosCon2023 OTTO 600 and OTTO 1500 Robots Gem  
#######################################################################
echo "Cloning RosCon2023 OTTO 600 and OTTO 1500 Robots Gem"
git clone --single-branch -b $ROSCON_DEMO_OTTO_ROBOTS_BRANCH $ROSCON_DEMO_OTTO_ROBOTS_REPO $ROSCON_DEMO_OTTO_ROBOTS_ROOT && \
    git -C $ROSCON_DEMO_OTTO_ROBOTS_ROOT lfs install && \
    git -C $ROSCON_DEMO_OTTO_ROBOTS_ROOT lfs pull && \
    git -C $ROSCON_DEMO_OTTO_ROBOTS_ROOT reset --hard $ROSCON_DEMO_OTTO_ROBOTS_COMMIT 
if [ $? -ne 0 ]
then
    echo "Error cloning RosCon2023 OTTO 600 and OTTO 1500 Robots Gem from $ROSCON_DEMO_OTTO_ROBOTS_REPO"
    exit 1
fi

$WORKSPACE/o3de/scripts/o3de.sh register -gp $ROSCON_DEMO_OTTO_ROBOTS_ROOT
if [ $? -ne 0 ]
then
    echo "Error registering the RosCon2023 OTTO 600 and OTTO 1500 Robots Gem"
    exit 1
fi


############################################################### \
# Clone and register the ROSCon2023Demo \
###############################################################
echo "Cloning the RosCon2023Demo project"
git clone --single-branch -b $ROSCON_DEMO_BRANCH $ROSCON_DEMO_REPO $ROSCON_DEMO_ROOT && \
    git -C $ROSCON_DEMO_ROOT lfs install && \
    git -C $ROSCON_DEMO_ROOT lfs pull && \
    git -C $ROSCON_DEMO_ROOT reset --hard $ROSCON_DEMO_COMMIT
if [ $? -ne 0 ]
then
    echo "Error cloning RosCon2023Demo project $ROSCON_DEMO_REPO"
    exit 1
fi

$WORKSPACE/o3de/scripts/o3de.sh register -pp $ROSCON_DEMO_PROJECT
if [ $? -ne 0 ]
then
    echo "Error registering the RosCon2023Demo Project"
    exit 1
fi


###############################################################################
# Track the git commits from all the repos
###############################################################################
echo -e "\n\
Repository                        | Commit    | Branch\n\
----------------------------------+-----------------------------------------\n\
o3de                              | $(git -C $O3DE_ROOT rev-parse --short HEAD)   | $O3DE_BRANCH\n\
o3de-extras                       | $(git -C $O3DE_EXTRAS_ROOT rev-parse --short HEAD)   | $O3DE_EXTRAS_BRANCH\n\
RosCon2023 Demo Human Worker Gem  | $(git -C $ROSCON_DEMO_HUMAN_WORKER_ROOT rev-parse --short HEAD)   | $ROSCON_DEMO_HUMAN_WORKER_BRANCH\n\
RosCon2023 UR Robots Gem          | $(git -C $ROSCON_DEMO_UR_ROBOTS_ROOT rev-parse --short HEAD)   | $ROSCON_DEMO_UR_ROBOTS_BRANCH\n\
RosCon2023 OTTO Robots Gem        | $(git -C $ROSCON_DEMO_OTTO_ROBOTS_ROOT rev-parse --short HEAD)   | $ROSCON_DEMO_OTTO_ROBOTS_BRANCH\n\
RosCon2023Demo Project            | $(git -C $ROSCON_DEMO_ROOT rev-parse --short HEAD)   | $ROSCON_DEMO_BRANCH\n\
\n\
" >> $WORKSPACE/git_commit.txt


###############################################################
# Generate the autoexec.cfg based on the selected level
###############################################################
echo -e "LoadLevel levels/$DEMO_LEVEL/$DEMO_LEVEL.spawnable\n\
r_displayInfo 0\n" > $ROSCON_DEMO_PROJECT/autoexec.cfg

###############################################################
# Build and install the additional ros2 packages and drivers
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
    echo "Error building and installing additional ros2 packages and drivers"
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
rm -rf $O3DE_ROOT
rm -rf $O3DE_EXTRAS_ROOT
rm -rf $ROSCON_DEMO_HUMAN_WORKER_ROOT
rm -rf $ROSCON_DEMO_UR_ROBOTS_ROOT
rm -rf $ROSCON_DEMO_OTTO_ROBOTS_ROOT
rm -rf $ROSCON_DEMO_PROJECT
rm -rf /root/.o3de

exit 0
