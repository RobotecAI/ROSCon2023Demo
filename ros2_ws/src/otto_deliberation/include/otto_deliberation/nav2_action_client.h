#pragma once

#include <blind_path_follower_msgs/action/follow_path.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include "otto_deliberation/tasks.h"
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
    template <typename ActionType>
    void SendSpecializedGoal(const typename ActionType::Goal& goal, ResultCallback callback, typename rclcpp_action::Client<ActionType>::SharedPtr m_client)
    {
        using GoalHandleType = rclcpp_action::ClientGoalHandle<ActionType>;
        auto sendGoalOptions = typename rclcpp_action::Client<ActionType>::SendGoalOptions();
        sendGoalOptions.goal_response_callback = [&logger = m_actionLogger, callback](std::shared_ptr<GoalHandleType> future)
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
        sendGoalOptions.feedback_callback = [](typename GoalHandleType::SharedPtr, const std::shared_ptr<const typename GoalHandleType::Feedback>)
        {
        };
        sendGoalOptions.result_callback = [&logger = m_actionLogger, callback](const typename GoalHandleType::WrappedResult& result)
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
        m_client->async_send_goal(goal, sendGoalOptions);
    }

    rclcpp_action::Client<Nav2Action>::SharedPtr m_nav2Client;
    rclcpp_action::Client<FollowPathAction>::SharedPtr m_followClient;

    rclcpp::Logger m_actionLogger;
};
