#!/bin/bash

if [ "$ROS_DISTRO" == "iron" ]
then
	git submodule init
	git submodule update
	
	cd ./src/Universal_Robots_ROS2_Driver/
	git checkout 129687763140508b4b5cb497b197ce1057defe88
	if [ $? -ne 0 ]
	then
		echo "Failed to checkout ROS 2 iron branch."
		exit 1
	fi
elif [ "$ROS_DISTRO" == "humble" ]
then
	# submodule init and update are sufficient for humble
	git submodule init
	git submodule update
	cd ./src/Universal_Robots_ROS2_Driver/
	git checkout humble
	if [ $? -ne 0 ]
	then
		echo "Failed to checkout ROS 2 humble branch."
		exit 1
	fi

elif [ -z "$ROS_DISTRO" ]
then
	echo "ROS 2 distribution not found."
	exit 1
else
	echo "ROS 2 $ROS_DISTRO is not supported in this project - switch to humble or iron distribution."
	exit 1
fi

echo "Submodules updated for ROS2 $ROS_DISTRO."
exit 0
