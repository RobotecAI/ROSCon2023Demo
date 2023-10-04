#include <blind_path_follower_msgs/action/follow_path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <otto_deliberation/tasks.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class Nav2ActionClient
{
public:
    using ResultCallback = std::function<void(bool)>;
    using Nav2Action = nav2_msgs::action::NavigateThroughPoses;
    using FollowPathAction = blind_path_follower_msgs::action::FollowPath;

    Nav2ActionClient(rclcpp::Node::SharedPtr node);
    void SendGoal(const NavPath& targetPath, ResultCallback resultCallback, bool goBlind = false, bool reverse = false, bool blindHighSpeed = false);

private:
    rclcpp_action::Client<Nav2Action>::SharedPtr m_nav2Client;
    rclcpp_action::Client<FollowPathAction>::SharedPtr m_followClient;

    void SendNav2Goal(const Nav2Action::Goal& goal, ResultCallback callback);
    void SendBlindGoal(const FollowPathAction::Goal& goal, ResultCallback callback);

    rclcpp::Logger m_actionLogger;
};
