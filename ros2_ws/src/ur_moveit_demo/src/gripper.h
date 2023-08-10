#include <control_msgs/action/gripper_command.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/client.hpp>
#include <rclcpp_action/create_client.hpp>
#include <string>
#include <control_msgs/action/detail/gripper_command__struct.hpp>

#pragma once

namespace Gripper
{
    class GripperController
    {
    public:
        GripperController(std::shared_ptr<rclcpp::Node> node, const std::string& topic);
        ~GripperController() = default;

        bool Grip();

        bool Release();

    private:
        bool SendGripCommand(bool shouldAttach);

        rclcpp_action::Client<control_msgs::action::GripperCommand>::SharedPtr m_client_ptr;

    };
} // namespace Gripper
