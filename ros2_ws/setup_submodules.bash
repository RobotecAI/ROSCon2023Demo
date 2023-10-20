#!/bin/bash

git submodule init
git submodule update

if [ "$ROS_DISTRO" == "iron" ]
then
	cd ./src/Universal_Robots_ROS2_Driver/
	git checkout 129687763140508b4b5cb497b197ce1057defe88
fi

echo "Submodules updated for ROS2 $ROS_DISTRO."
