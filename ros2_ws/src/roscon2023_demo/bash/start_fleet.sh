#! /bin/bash

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`

# ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_1', initial_pose: {position: {x: -32.0 , y: 2, z: 0.1}}}"

export TaskDir=${SCRIPTPATH}/../../otto_deliberation/config/


${SCRIPTPATH}/navigate_deliberation.sh otto_1 Line1 ${TaskDir}/otto600_tasks_12.yaml
sleep 3

${SCRIPTPATH}/navigate_deliberation.sh otto_2 Line2 ${TaskDir}/otto600_tasks_12.yaml
sleep 3

${SCRIPTPATH}/navigate_deliberation.sh otto_3 Line3 ${TaskDir}/otto600_tasks_34.yaml
sleep 3

${SCRIPTPATH}/navigate_deliberation.sh otto_4 Line4 ${TaskDir}/otto600_tasks_34.yaml
sleep 3


while true; do
    read -n 1 -p "Press 'q' to exit: " input
    if [ "$input" == "q" ]; then
        ${SCRIPTPATH}/stop_fleet.sh
        echo ""
        break
    fi
done
