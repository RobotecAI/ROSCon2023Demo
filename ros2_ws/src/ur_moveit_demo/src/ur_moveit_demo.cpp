#include <memory>

#include <thread> 
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <iostream>


struct Transform {
  double x = 0;
  double y = 0;
  double z = 0;
} box_tf;


 geometry_msgs::msg::Pose box_pose =  geometry_msgs::msg::Pose();


void tf_callback(tf2_msgs::msg::TFMessage mess) {

  for (auto tf : mess.transforms) 
  {

    if(tf.child_frame_id != "Box1/box")
    {
      continue;
    }

    box_pose.orientation = tf.transform.rotation; 
    auto tr = tf.transform.translation;



    box_pose.position.x = tr.x;
    box_pose.position.y = tr.y;
    box_pose.position.z = tr.z + 0.3641039 * 0.28449010848999023;



    box_tf.x = tr.x;
    box_tf.y = tr.y;
    box_tf.z = tr.z + 0.3641039  * 0.28449010848999023;

    // std::cerr << "got box transform: " << tr.x << ", " << tr.y << ", " << tr.z <<"\n";
  }
}



int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("ur_moveit_demo");

  // Spin up a SingleThreadedExecutor for MoveItVisualTools to interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });


  auto tf_subscribtion = node->create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, tf_callback) ;


  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
      node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
      move_group_interface.getRobotModel()};

  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();


  move_group_interface.setNumPlanningAttempts(5);
  move_group_interface.setMaxVelocityScalingFactor(0.5);


  // Create closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text) {
    auto const text_pose = [] {
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;  // Place text 1m above the base link
      return msg;
    }();
    moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
                                    rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text) {
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
      jmg = move_group_interface.getRobotModel()->getJointModelGroup(
          "ur_manipulator")](auto const trajectory) {
        moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
      };


  using namespace std::chrono_literals;

  std::this_thread::sleep_for(2000ms);

  
  std::cerr << "Box Pose: " << box_pose.position.x << ", " << box_pose.position.y << ", " << box_pose.position.z << "\n"; 


  // Add box to the environment
  auto const box_1 = [frame_id =
                                  move_group_interface.getPlanningFrame(), &box_pose] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);

                    
    primitive.dimensions[primitive.BOX_X] = 0.24811747670173645 * 1.0171222686767578;
    primitive.dimensions[primitive.BOX_Y] = 0.42500990629196167 * 0.511430025100708;
    primitive.dimensions[primitive.BOX_Z] = 0.28449010848999023 * 0.7279044985771179;

    // Define the pose of the box (relative to the frame_id)
    // geometry_msgs::msg::Pose box_pose;


    // box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    // box_pose.position.x = 0.0;
    // box_pose.position.y = 0.0;
    // box_pose.position.z = -0.25;

     std::cerr << "Box Pose: " << box_pose.position.x << ", " << box_pose.position.y << ", " << box_pose.position.z << "\n"; 

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();






  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 1.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = 0.0;
    msg.orientation.w = 0.0;
    // msg.position.x = 0.7;
    // msg.position.y = 0.4;
    // msg.position.z = 0.25;
    msg.position.x = box_tf.x;
    msg.position.y = box_tf.y;
    msg.position.z = box_tf.z + 0.25;
    return msg;
  }();

  std::cerr << "Goal Pose: " << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << "\n"; 

  move_group_interface.setPoseTarget(target_pose);


  // Create collision object for the robot to avoid
  auto const collision_object_1 = [frame_id =
                                  move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "obstacle1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.5;
    primitive.dimensions[primitive.BOX_Y] = 1.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.27;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // auto const collision_object_2 = [frame_id =
  //                                 move_group_interface.getPlanningFrame()] {
  //   moveit_msgs::msg::CollisionObject collision_object;
  //   collision_object.header.frame_id = frame_id;
  //   collision_object.id = "obstacle2";
  //   shape_msgs::msg::SolidPrimitive primitive;

  //   // Define the size of the box in meters
  //   primitive.type = primitive.BOX;
  //   primitive.dimensions.resize(3);
  //   primitive.dimensions[primitive.BOX_X] = 0.1;
  //   primitive.dimensions[primitive.BOX_Y] = 0.1;
  //   primitive.dimensions[primitive.BOX_Z] = 0.3;

  //   // Define the pose of the box (relative to the frame_id)
  //   geometry_msgs::msg::Pose box_pose;
  //   box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
  //   box_pose.position.x = 0.6;
  //   box_pose.position.y = 0.0;
  //   box_pose.position.z = 0.3;

  //   collision_object.primitives.push_back(primitive);
  //   collision_object.primitive_poses.push_back(box_pose);
  //   collision_object.operation = collision_object.ADD;

  //   return collision_object;
  // }();

  // Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // planning_scene_interface.applyCollisionObjects({collision_object_2, collision_object_1});

  planning_scene_interface.applyCollisionObjects({collision_object_1, box_1});

  // planning_scene_interface.applyCollisionObject(collision_object_1);


  // Create a plan to that target pose
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success) {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
    
    rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
    spinner.join();  // <--- Join the thread before exiting
    return 0;
  }


  // auto node = rclcpp::Node::make_shared("gripper_client");


    auto client_ptr =  rclcpp_action::create_client<control_msgs::action::GripperCommand>(node, "/gripper_server");


    if (!client_ptr->wait_for_action_server()) {
        RCLCPP_ERROR(logger, "Action server not available after waiting");
        // Shutdown ROS
        rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
        spinner.join();  // <--- Join the thread before exiting
        return 0;
    }



  auto goal = control_msgs::action::GripperCommand::Goal();
  goal.command.position = 0.0;
  goal.command.max_effort = 10000.0;
  auto future = client_ptr->async_send_goal(goal);
  future.wait();

  move_group_interface.attachObject(box_1.id, "gripper_link");
  prompt("Press 'next' in the RvizVisualToolsGui window once the new object is attached to the robot");

  // auto const target_pose_2 = [] {
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.x = 1.0;
  //   msg.orientation.y = 0.0;
  //   msg.orientation.z = 0.0;
  //   msg.orientation.w = 0.0;
  //   msg.position.x = 0.7;
  //   msg.position.y = -0.4;
  //   msg.position.z = 0.25;
  //   return msg;
  // }();

  auto target_pose_2 = target_pose;
  target_pose_2.position.x *= -1;
  target_pose_2.position.y *= -1;


  move_group_interface.setPoseTarget(target_pose_2);
  prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  draw_title("Planning");
  moveit_visual_tools.trigger();
  auto const [success_2, plan_2] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if (success_2) {
    draw_trajectory_tool_path(plan_2.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();
    move_group_interface.execute(plan_2);
  } else {
    draw_title("Planning Failed!");
    moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }


  // Shutdown ROS
  rclcpp::shutdown();  // <--- This will cause the spin function in the thread to return
  spinner.join();  // <--- Join the thread before exiting
  return 0;
}


// class Demo {
//   public:


//     Demo(int argc, char * argv[]) 
//     {

//     }
 
//   private:

//   std::shared_ptr<rclcpp::Node> node;

// };