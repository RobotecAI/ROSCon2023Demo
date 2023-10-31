#! /bin/sh

export ROBOT_NAMESPACE=$1
export LINE=$2
export TASK=$3
echo Staring robot $ROBOT_NAMESPACE
# kill -9 $(ps -aux | grep $ROBOT_NAMESPACE | awk '{print $2}')

ros2 service call /lock_service lock_service_msgs/srv/Lock "{ key: '${LINE}_GoToPickup', lock_status: false }"

screen -S ${ROBOT_NAMESPACE}_nav2 -dm ros2 launch otto_fleet_nav otto_fleet_nav_launch.py spawn_only_one:="${ROBOT_NAMESPACE}" fleet_config_path:=/home/michalpelka/github/ROSCon2023Demo/ros2_ws/src/roscon2023_demo/config/ROSCon2023Config_900.yaml use_rviz:=True

screen -S  ${ROBOT_NAMESPACE}_deliberation -dm ros2 launch otto_deliberation otto_deliberation.launch.py namespace:="${ROBOT_NAMESPACE}" assigned_lane:="${LINE}" tasks_config_file:=$TASK
