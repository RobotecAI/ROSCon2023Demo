
#include <chrono>
#include <lock_service_msgs/srv/detail/lock__struct.hpp>
#include <otto_deliberation/nav2_action_client.h>
#include <otto_deliberation/robot_status.h>
#include <otto_deliberation/tasks.h>
#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>

#pragma once

class OttoAutonomy
{
public:
    OttoAutonomy(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr lock_node);
    void SetTasks(const RobotTasks& tasks, bool loop = true);
    void SetLane(const std::string& lane_name);
    void Update();

    void NavigationGoalCompleted(bool success);
    void NotifyCargoChanged(bool hasCargoNow);
    RobotStatus GetCurrentStatus() const;
    std::string GetCurrentOperationDescription() const;
    std::string GetCurrentTaskName() const;

private:
    bool SendLockRequest(const std::string& path_name, bool lock_status);

    rclcpp::Logger m_logger;
    rclcpp::Client<lock_service_msgs::srv::Lock>::SharedPtr m_lockServiceClient;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr m_lifterPublisher;
    RobotStatus m_robotStatus;
    Nav2ActionClient m_nav2ActionClient;
    std::string m_laneName;
    std::chrono::time_point<std::chrono::system_clock> m_waitTimePointPreTaskDelay;
    std::chrono::time_point<std::chrono::system_clock> m_waitTimePointPostTaskDelay;

    RobotTasks m_robotTasks;
    bool m_isWaitingPostTaskDelay{ false };
    bool m_isWaitingPreTaskDelay{ false };
    bool m_loop{ false };
    bool m_hasLock{ false };
    std::string m_lockTaskName {};
    std::string m_currentOperationDescription;
};
