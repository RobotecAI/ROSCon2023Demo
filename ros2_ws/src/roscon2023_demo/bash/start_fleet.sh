#! /bin/sh

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_1', initial_pose: {position: {x: -32.0 , y: 2, z: 0.1}}}"

export TaskDir=/home/michalpelka/github/ROSCon2023Demo/ros2_ws/src/otto_deliberation/config/
./navigate_deliberation.sh otto_1 Line1 ${TaskDir}/otto600_tasks_12.yaml
