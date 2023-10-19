


Package providing MoveIt control of Universal Robots present in the Demo.


Usage:

1. Build package and source `install/setup.bash`
2. Open Editor and run the simulation.
3. (In separate terminal) Launch MoveIt control nodes: `ros2 launch ur_moveit_demo mtc.launch.py ur_namespace:=ur1`  
(the namespace can be changed to control different robots)
4. You may then run `ros2 launch ur_moveit_demo rviz.launch.py` in separate terminal to open RViz2 showing all arm movements.
5. To start the palletization use `ros2 action send_goal /ur1/MTC ur_moveit_demo_msg/action/Mtc "{num_of_boxes: 3}"` where num_of_boxes is the amount of boxes placed on the pallet.
