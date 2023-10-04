#include "gripper.h"

namespace Gripper
{
    GripperController::GripperController(std::shared_ptr<rclcpp::Node> node, const std::string& topic)
    {
        m_client_ptr = rclcpp_action::create_client<control_msgs::action::GripperCommand>(node, topic);
    }

    bool GripperController::SendGripCommand(bool shouldAttach)
    {
        auto goal = control_msgs::action::GripperCommand::Goal();
        goal.command.position = shouldAttach ? 0.0 : 1.0;
        goal.command.max_effort = 10000.0;
        auto future = m_client_ptr->async_send_goal(goal);

        future.wait();

        auto goal_handle = future.get();

        auto result_future = m_client_ptr->async_get_result(goal_handle);

        result_future.wait();

        auto result = result_future.get().result;

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
