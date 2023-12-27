#include "gripper.h"

#include <rclcpp_action/create_client.hpp>

namespace Gripper
{
    GripperController::GripperController(std::shared_ptr<rclcpp::Node> node, const std::string& topic)
    {
        m_client = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node, topic);
    }

    bool GripperController::SendGripCommand(bool shouldAttach)
    {
        auto goal = control_msgs::action::GripperCommand::Goal();
        goal.command.position = shouldAttach ? 0.0 : 1.0;
        goal.command.max_effort = 10000.0;

        auto future = m_client->async_send_goal(goal);
        auto goalHandle = future.get();

        auto resultFuture = m_client->async_get_result(goalHandle);
        auto result = resultFuture.get().result;

        return result->reached_goal;
    }

    bool GripperController::Grip()
    {
        return SendGripCommand(true);
    }

    bool GripperController::Release()
    {
        return SendGripCommand(false);
    }
} // namespace Gripper
