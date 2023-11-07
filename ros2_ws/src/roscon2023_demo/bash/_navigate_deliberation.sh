#! /bin/sh

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`

export ROBOT_NAMESPACE=$1
export LINE=$2
export TASK=$3
echo Staring robot $ROBOT_NAMESPACE

ros2 service call /lock_service lock_service_msgs/srv/Lock "{ key: '${LINE}_GoToPickup', lock_status: false }"

screen -S ${ROBOT_NAMESPACE}_nav2 -dm ros2 launch otto_fleet_nav otto_fleet_nav_launch.py spawn_only_one:="${ROBOT_NAMESPACE}" fleet_config_path:=${SCRIPTPATH}/../config/ROSCon2023Config_900.yaml use_rviz:=False
screen -S ${ROBOT_NAMESPACE}_deliberation -dm ros2 launch otto_deliberation otto_deliberation.launch.py namespace:="${ROBOT_NAMESPACE}" assigned_lane:="${LINE}" tasks_config_file:=$TASK
