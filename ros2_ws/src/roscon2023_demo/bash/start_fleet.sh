#! /bin/sh

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`

# ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_1', initial_pose: {position: {x: -32.0 , y: 2, z: 0.1}}}"

export TaskDir=${SCRIPTPATH}/../../otto_deliberation/config/


# Group 1

${SCRIPTPATH}/_navigate_deliberation.sh otto_1 Line1 ${TaskDir}/otto600_tasks_12.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_2 Line1 ${TaskDir}/otto600_tasks_12.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_3 Line2 ${TaskDir}/otto600_tasks_12.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_4 Line2 ${TaskDir}/otto600_tasks_12.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_5 Line3 ${TaskDir}/otto600_tasks_34.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_6 Line3 ${TaskDir}/otto600_tasks_34.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_7 Line4 ${TaskDir}/otto600_tasks_34.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_8 Line4 ${TaskDir}/otto600_tasks_34.yaml
sleep 3

echo "starting group 2"

${SCRIPTPATH}/_navigate_deliberation.sh otto_21 Line21 ${TaskDir}/otto600_tasks_12_lane2.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_22 Line21 ${TaskDir}/otto600_tasks_12_lane2.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_23 Line22 ${TaskDir}/otto600_tasks_12_lane2.yaml 
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_24 Line22 ${TaskDir}/otto600_tasks_12_lane2.yaml 
sleep 3



${SCRIPTPATH}/_navigate_deliberation.sh otto_25 Line23 ${TaskDir}/otto600_tasks_34_lane2.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_26 Line23 ${TaskDir}/otto600_tasks_34_lane2.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_27 Line24 ${TaskDir}/otto600_tasks_34_lane2.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_28 Line24 ${TaskDir}/otto600_tasks_34_lane2.yaml
sleep 3

echo "starting group 3"

${SCRIPTPATH}/_navigate_deliberation.sh otto_31 Line31 ${TaskDir}/otto600_tasks_12_lane3.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_32 Line31 ${TaskDir}/otto600_tasks_12_lane3.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_33 Line32 ${TaskDir}/otto600_tasks_12_lane3.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_34 Line32 ${TaskDir}/otto600_tasks_12_lane3.yaml
sleep 3


${SCRIPTPATH}/_navigate_deliberation.sh otto_35 Line33 ${TaskDir}/otto600_tasks_34_lane3.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_36 Line33 ${TaskDir}/otto600_tasks_34_lane3.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_37 Line34 ${TaskDir}/otto600_tasks_34_lane3.yaml
sleep 3

${SCRIPTPATH}/_navigate_deliberation.sh otto_38 Line34 ${TaskDir}/otto600_tasks_34_lane3.yaml
sleep 3

while true; do
    read -n 1 -p "Press 'q' to exit: " input
    if [ "$input" == "q" ]; then
        ${SCRIPTPATH}/stop_fleet.sh
        echo ""
        break
    fi
done
