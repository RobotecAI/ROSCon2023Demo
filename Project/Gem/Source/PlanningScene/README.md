
# ShapePublisherComponent

Component publishing ROS2 description of Shapes.


## Interface

The interface used is `moveit_msgs::msg::CollisionObject`, that can be used by MoveIt2 in Planning Scene for collision-aware motion planning.

Default topic used by MoveIt2 to receive those messages is `<namespace>/collision_object`.



## Supported Shapes

Currently supported shapes are BoxShape and CylinderShape (Note: AxisAlignedBox is not supported).



## Example

The following commands show how to view the shapes in MoveIt2 in multi-robot setup:

`ros2 launch ur_moveit_demo rviz.launch.py ur_namespace:=ur1 ur_type:=ur10 rviz_config_file:=collision_objects.rviz`

`ros2 launch ur_moveit_demo rviz.launch.py ur_namespace:=ur2 ur_type:=ur10 rviz_config_file:=collision_objects.rviz`

You may then interact with those shapes in `Motion Planning` window.



## Additional information

 - Shape and Robot should be in the same tf tree (MoveIt2 must calculate the position in planning frame, set in robot's description)

 - The `object id` must be unique for each shape for MoveIt2 to process it correctly.

 - `Namespace` in the component and `ur_namespace` should match so that the topic name is the same.
