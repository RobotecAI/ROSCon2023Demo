#pragma once

#include <control_msgs/action/detail/gripper_command__struct.hpp>
#include <control_msgs/action/gripper_command.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>

#include <memory>
#include <string>

namespace Gripper
{
    class GripperController
    {
    public:
        GripperController(std::shared_ptr<rclcpp::Node> node, const std::string& topic);

        bool Grip();
        bool Release();

    private:
        bool SendGripCommand(bool shouldAttach);

        rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr m_client;
    };
} // namespace Gripper
