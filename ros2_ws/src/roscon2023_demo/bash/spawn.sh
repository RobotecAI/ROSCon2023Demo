#! /bin/sh

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`


export ConfigDir=${SCRIPTPATH}/../../roscon2023_demo/config/


echo Starting MoveIt stacks and deliberation locks

screen -S MOVEIT_LOCKS -dm ros2 launch roscon2023_demo ROSCon2023Demo.launch.py ROS2Con2023Config:=${ConfigDir}/ROSCon2023Config_900.yaml start_amr_navigation:=False spawn_amrs:=False

ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_1', entity_namespace: 'otto_1', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -32.0 , y: 2, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_2', entity_namespace: 'otto_2', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -30.0 , y: 2, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_3', entity_namespace: 'otto_3', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -28.0 , y: 2, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_4', entity_namespace: 'otto_4', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -26.0 , y: 2, z: 0.1}}}}"

ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_5', entity_namespace: 'otto_5', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -32.0 , y: 27, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_6', entity_namespace: 'otto_6', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -30.0 , y: 27, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_7', entity_namespace: 'otto_7', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -28.0 , y: 27, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_8', entity_namespace: 'otto_8', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -26.0 , y: 27, z: 0.1}}}}"


ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_21', entity_namespace: 'otto_21', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -32.0 , y: 32, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_22', entity_namespace: 'otto_22', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -30.0 , y: 32, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_23', entity_namespace: 'otto_23', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -28.0 , y: 32, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_24', entity_namespace: 'otto_24', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -26.0 , y: 32, z: 0.1}}}}"

ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_25', entity_namespace: 'otto_25', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -32.0 , y: 57, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_26', entity_namespace: 'otto_26', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -30.0 , y: 57, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_27', entity_namespace: 'otto_27', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -28.0 , y: 57, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_28', entity_namespace: 'otto_28', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -26.0 , y: 57, z: 0.1}}}}"

ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_31', entity_namespace: 'otto_31', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -32.0 , y: 62, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_32', entity_namespace: 'otto_32', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -30.0 , y: 62, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_33', entity_namespace: 'otto_33', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -28.0 , y: 62, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_34', entity_namespace: 'otto_34', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -26.0 , y: 62, z: 0.1}}}}"

ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_35', entity_namespace: 'otto_35', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -32.0 , y: 87, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_36', entity_namespace: 'otto_36', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -30.0 , y: 87, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_37', entity_namespace: 'otto_37', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -28.0 , y: 87, z: 0.1}}}}"
ros2 service call /spawn_entity simulation_interfaces/srv/SpawnEntity "{name: 'otto_38', entity_namespace: 'otto_38', uri: 'product_asset:///prefabs/otto600/otto600withpallet.spawnable', initial_pose: {pose: {position: {x: -26.0 , y: 87, z: 0.1}}}}"

