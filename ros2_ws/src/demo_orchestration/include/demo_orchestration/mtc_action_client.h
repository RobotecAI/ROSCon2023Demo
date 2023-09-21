#include <nav2_msgs/action/detail/navigate_through_poses__struct.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ur_moveit_demo_msg/action/follow_path.hpp>
#include <ur_moveit_demo_msg/action/mtc.hpp>
#include <functional>

class MTCActionClient {
    using ResultCallback = std::function<void(bool)>;
	using MTCAction = ur_moveit_demo_msg::action::Mtc;
	using GoalHandleMTC = rclcpp_action::ClientGoalHandle<MTCAction>; 
}