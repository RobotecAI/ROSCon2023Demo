#include <nav2_msgs/action/detail/navigate_through_poses__struct.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <nav_msgs/msg/path.hpp>
#include <otto_deliberation/nav2_action_client.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/create_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

Nav2ActionClient::Nav2ActionClient(rclcpp::Node::SharedPtr node)
    : m_actionLogger(node->get_logger())
{
    std::string ns(node->get_namespace());
    if (ns == "/")
    {
        RCLCPP_ERROR(m_actionLogger, "This node must be run in a namespace, terminating");
        std::abort();
    }
    m_nav2Client = rclcpp_action::create_client<Nav2Action>(node, ns + "/navigate_through_poses");
    m_followClient = rclcpp_action::create_client<FollowPathAction>(node, ns + "/blind_follow_path");
}

void Nav2ActionClient::SendNav2Goal(const Nav2Action::Goal& goal_msg, ResultCallback callback)
{
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Action>;
    auto send_goal_options = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&logger = m_actionLogger](std::shared_ptr<GoalHandleNav2> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(logger, "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(logger, "Goal accepted by server, waiting for result");
        }
    };
    send_goal_options.feedback_callback = [](GoalHandleNav2::SharedPtr, const std::shared_ptr<const GoalHandleNav2::Feedback>)
    {
    };
    send_goal_options.result_callback = [&logger = m_actionLogger, callback](const GoalHandleNav2::WrappedResult& result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            callback(true);
            break;
        default:
            RCLCPP_ERROR(logger, "Goal did not succeed");
            callback(false);
            return;
        };
    };
    m_nav2Client->async_send_goal(goal_msg, send_goal_options);
}

// TODO - template with action type instead!
void Nav2ActionClient::SendBlindGoal(const FollowPathAction::Goal& goal_msg, ResultCallback callback)
{
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPathAction>;
    auto send_goal_options = rclcpp_action::Client<FollowPathAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [&logger = m_actionLogger](std::shared_ptr<GoalHandleFollowPath> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(logger, "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(logger, "Goal accepted by server, waiting for result");
        }
    };
    send_goal_options.feedback_callback = [](GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const GoalHandleFollowPath::Feedback>)
    {
    };
    send_goal_options.result_callback = [&logger = m_actionLogger, callback](const GoalHandleFollowPath::WrappedResult& result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            callback(true);
            break;
        default:
            RCLCPP_ERROR(logger, "Goal did not succeed");
            callback(false);
            return;
        };
    };
    m_followClient->async_send_goal(goal_msg, send_goal_options);
}

void Nav2ActionClient::SendGoal(const NavPath& targetPath, std::function<void(bool)> completionCallback, bool goBlind, bool reverse, bool highSpeed)
{
    if (goBlind)
    {
        if (!m_followClient->wait_for_action_server())
        {
            RCLCPP_ERROR(m_actionLogger, "Action server not available after waiting");
            return;
        }

        const float speed = highSpeed ? 0.55 : 0.25;
        auto goal_msg = FollowPathAction::Goal();
        goal_msg.speed = speed;
        goal_msg.reverse = reverse;
        goal_msg.poses = targetPath.poses;
        SendBlindGoal(goal_msg, completionCallback);
        return;
    }

    if (!m_nav2Client->wait_for_action_server())
    {
        RCLCPP_ERROR(m_actionLogger, "Action server not available after waiting");
        return;
    }

    auto goal_msg = Nav2Action::Goal();
    goal_msg.poses = targetPath.poses;
    SendNav2Goal(goal_msg, completionCallback);
}
