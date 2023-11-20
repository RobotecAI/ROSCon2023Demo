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

void Nav2ActionClient::SendNav2Goal(const Nav2Action::Goal& goal, ResultCallback callback)
{
    using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<Nav2Action>;
    auto sendGoalOptions = rclcpp_action::Client<Nav2Action>::SendGoalOptions();
    sendGoalOptions.goal_response_callback = [&logger = m_actionLogger, callback](std::shared_ptr<GoalHandleNav2> future)
    {
        auto goalHandle = future.get();
        if (!goalHandle)
        {
            RCLCPP_ERROR(logger, "Goal was rejected by server");
            callback(false);
        }
        else
        {
            RCLCPP_INFO(logger, "Goal accepted by server, waiting for result");
        }
    };
    sendGoalOptions.feedback_callback = [](GoalHandleNav2::SharedPtr, const std::shared_ptr<const GoalHandleNav2::Feedback>)
    {
    };
    sendGoalOptions.result_callback = [&logger = m_actionLogger, callback](const GoalHandleNav2::WrappedResult& result)
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
    m_nav2Client->async_send_goal(goal, sendGoalOptions);
}

// TODO - template with action type instead!
void Nav2ActionClient::SendBlindGoal(const FollowPathAction::Goal& goal, ResultCallback callback)
{
    using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPathAction>;
    auto sendGoalOptions = rclcpp_action::Client<FollowPathAction>::SendGoalOptions();
    sendGoalOptions.goal_response_callback = [&logger = m_actionLogger](std::shared_ptr<GoalHandleFollowPath> future)
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
    sendGoalOptions.feedback_callback = [](GoalHandleFollowPath::SharedPtr, const std::shared_ptr<const GoalHandleFollowPath::Feedback>)
    {
    };
    sendGoalOptions.result_callback = [&logger = m_actionLogger, callback](const GoalHandleFollowPath::WrappedResult& result)
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
    m_followClient->async_send_goal(goal, sendGoalOptions);
}

void Nav2ActionClient::SendGoal(const NavPath& targetPath, ResultCallback completionCallback, bool goBlind, bool reverse, bool highSpeed)
{
    if (goBlind)
    {
        if (!m_followClient->wait_for_action_server())
        {
            RCLCPP_ERROR(m_actionLogger, "Action server not available after waiting");
            return;
        }

        const float speed = highSpeed ? 0.55 : 0.25;
        auto goal = FollowPathAction::Goal();
        goal.speed = speed;
        goal.reverse = reverse;
        goal.poses = targetPath.poses;
        SendBlindGoal(goal, completionCallback);
        return;
    }

    if (!m_nav2Client->wait_for_action_server())
    {
        RCLCPP_ERROR(m_actionLogger, "Action server not available after waiting");
        return;
    }

    auto goal = Nav2Action::Goal();
    goal.poses = targetPath.poses;
    SendNav2Goal(goal, completionCallback);
}
