#! /bin/sh

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`


export ConfigDir=${SCRIPTPATH}/../../roscon2023_demo/config/


echo Starting MoveIt stacks and deliberation locks

screen -S MOVEIT_LOCKS -dm ros2 launch roscon2023_demo ROSCon2023Demo.launch.py ROS2Con2023Config:=${ConfigDir}/ROSCon2023Config_900.yaml start_amr_navigation:=False spawn_amrs:=False

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_1', initial_pose: {position: {x: -32.0 , y: 2, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_2', initial_pose: {position: {x: -30.0 , y: 2, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_3', initial_pose: {position: {x: -28.0 , y: 2, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_4', initial_pose: {position: {x: -26.0 , y: 2, z: 0.1}}}"

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_5', initial_pose: {position: {x: -32.0 , y: 27, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_6', initial_pose: {position: {x: -30.0 , y: 27, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_7', initial_pose: {position: {x: -28.0 , y: 27, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_8', initial_pose: {position: {x: -26.0 , y: 27, z: 0.1}}}"


ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_21', initial_pose: {position: {x: -32.0 , y: 32, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_22', initial_pose: {position: {x: -30.0 , y: 32, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_23', initial_pose: {position: {x: -28.0 , y: 32, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_24', initial_pose: {position: {x: -26.0 , y: 32, z: 0.1}}}"

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_25', initial_pose: {position: {x: -32.0 , y: 57, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_26', initial_pose: {position: {x: -30.0 , y: 57, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_27', initial_pose: {position: {x: -28.0 , y: 57, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_28', initial_pose: {position: {x: -26.0 , y: 57, z: 0.1}}}"

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_31', initial_pose: {position: {x: -32.0 , y: 62, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_32', initial_pose: {position: {x: -30.0 , y: 62, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_33', initial_pose: {position: {x: -28.0 , y: 62, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_34', initial_pose: {position: {x: -26.0 , y: 62, z: 0.1}}}"

ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_35', initial_pose: {position: {x: -32.0 , y: 87, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_36', initial_pose: {position: {x: -30.0 , y: 87, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_37', initial_pose: {position: {x: -28.0 , y: 87, z: 0.1}}}"
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "{name: 'otto600', robot_namespace: 'otto_38', initial_pose: {position: {x: -26.0 , y: 87, z: 0.1}}}"

