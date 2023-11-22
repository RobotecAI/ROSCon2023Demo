#pragma once

#include <lock_service_msgs/srv/detail/lock__struct.hpp>
#include "otto_deliberation/nav2_action_client.h"
#include "otto_deliberation/robot_status.h"
#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <string>

class OttoAutonomy
{
public:
    OttoAutonomy(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr lockNode);
    void SetTasks(const RobotTasks& tasks);
    void SetLane(const std::string& laneName);
    void Update();

    void NavigationGoalCompleted(bool success);
    void NotifyCargoChanged(bool hasCargoNow);
    RobotStatus GetCurrentStatus() const;
    std::string GetCurrentOperationDescription() const;
    std::string GetCurrentTaskName() const;

private:
    bool SendLockRequest(const std::string& pathName, bool lockStatus);
    void SendColor(const std::string& color);

    rclcpp::Logger m_logger;
    rclcpp::Clock m_clock;

    rclcpp::Client<lock_service_msgs::srv::Lock>::SharedPtr m_lockServiceClient;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_lifterPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_colorPublisher;
    RobotStatus m_robotStatus;
    Nav2ActionClient m_nav2ActionClient;
    std::string m_laneName;
    rclcpp::Time m_waitTimePointPreTaskDelay;
    rclcpp::Time m_waitTimePointPostTaskDelay;
    rclcpp::Time m_startNavigationTimePoint;
    RobotTasks m_robotTasks;
    bool m_isWaitingPostTaskDelay{ false };
    bool m_isWaitingPreTaskDelay{ false };
    bool m_hasLock{ false };
    std::string m_lockTaskName {};
    std::string m_currentOperationDescription;
};
