#include "otto_deliberation/nav2_action_client.h"

#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp_action/create_client.hpp>

Nav2ActionClient::Nav2ActionClient(rclcpp::Node::SharedPtr node)
    : m_actionLogger(node->get_logger())
{
    const std::string nodeNamespace(node->get_namespace());
    if (nodeNamespace == "/")
    {
        RCLCPP_ERROR(m_actionLogger, "This node must be run in a namespace, terminating");
        std::abort();
    }
    m_nav2Client = rclcpp_action::create_client<Nav2Action>(node, "navigate_through_poses");
    m_followClient = rclcpp_action::create_client<FollowPathAction>(node, "blind_follow_path");
}

void Nav2ActionClient::SendGoal(const NavPath& targetPath, ResultCallback completionCallback, bool goBlind, bool reverse, bool highSpeed)
{
    if (!m_followClient->wait_for_action_server())
    {
        RCLCPP_ERROR(m_actionLogger, "Action server not available after waiting");
        return;
    }

    if (goBlind)
    {
        constexpr float highSpeedValue = 0.55f;
        constexpr float lowSpeedValue = 0.25f;

        auto goal = FollowPathAction::Goal();
        goal.speed = highSpeed ? highSpeedValue : lowSpeedValue;
        goal.reverse = reverse;
        goal.poses = targetPath.poses;
        SendSpecializedGoal<FollowPathAction>(goal, completionCallback, m_followClient);
    }
    else
    {
        auto goal = Nav2Action::Goal();
        goal.poses = targetPath.poses;
        SendSpecializedGoal<Nav2Action>(goal, completionCallback, m_nav2Client);
    }
}
