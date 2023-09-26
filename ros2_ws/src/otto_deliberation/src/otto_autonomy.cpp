
#include "otto_deliberation/robot_status.h"
#include "otto_deliberation/tasks.h"
#include <chrono>
#include <lock_service_msgs/srv/detail/lock__struct.hpp>
#include <lock_service_msgs/srv/lock.hpp>
#include <otto_deliberation/otto_autonomy.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>


OttoAutonomy::OttoAutonomy(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr lock_node)
    : m_logger(node->get_logger())
    , m_nav2ActionClient(node)
{
    std::string lock_service = lock_node->declare_parameter<std::string>("lock_service", "/lock_service");
    lock_node->get_parameter<std::string>("lock_service", lock_service);
    m_lockServiceClient = lock_node->create_client<lock_service_msgs::srv::Lock>(lock_service);
    if (!m_lockServiceClient->wait_for_service(std::chrono::seconds(2)))
    {
        RCLCPP_ERROR(lock_node->get_logger(), "Lock service named %s not available", lock_service.c_str());
    }

    m_lifterPublisher = node->create_publisher<std_msgs::msg::Bool>("lifter", 10);
}

void OttoAutonomy::SetTasks(const RobotTasks& tasks, bool loop)
{
    m_loop = loop;
    m_robotTasks = tasks;
    m_robotStatus.m_currentTask = m_robotTasks.GetTasks().front(); // set first task as current task
}

void OttoAutonomy::SetLane(const std::string& lane_name)
{
    m_laneName = lane_name;
}

RobotStatus OttoAutonomy::GetCurrentStatus() const
{
    return m_robotStatus;
}

bool OttoAutonomy::SendLockRequest(const std::string& path_name, bool lock_status)
{
    auto lockRequest = std::make_shared<lock_service_msgs::srv::Lock::Request>();
    lockRequest->lane_name = m_laneName;
    lockRequest->path_name = path_name;
    lockRequest->lock_status = lock_status;
    auto future = m_lockServiceClient->async_send_request(lockRequest);
    future.wait();
    auto result = future.get();
    return result->result;
}

void OttoAutonomy::Update()
{
    std_msgs::msg::Bool lifterStatus;
    m_lifterPublisher->publish(lifterStatus);

    if (m_robotTasks.GetTasks().empty())
    {
        return;
    }

    const auto currentTaskKey = m_robotStatus.m_currentTask;
    const auto& currentTask = m_robotTasks.ConstructTask(currentTaskKey);

    // try to lock
    if (currentTask.m_isNeedLock)
    {
        if (!SendLockRequest(currentTask.m_taskName, true))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting to lock %s", currentTaskKey.c_str());
            return;
        }
    }

    // wait for pre-task delay
    if (currentTask.m_preTaskDelay) {
        if (!m_isWaiting) {
            m_isWaiting = true;
            m_waitTimePoint = std::chrono::system_clock::now();
        }
        if ((std::chrono::system_clock::now() - m_waitTimePoint).count() < currentTask.m_preTaskDelay.value())
        {
            // Waiting for pre-task delay
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting to predelay %s", currentTaskKey.c_str());
            return;
        }
    }

    if (currentTask.m_isCargoLoad)
    {
        if ( m_robotStatus.m_cargoStatus != RobotCargoStatus::CARGO_LOADED)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting to load %s", currentTaskKey.c_str());
            return;
        }

    }

    if (currentTask.m_isCargoUnload)
    {
        if ( m_robotStatus.m_cargoStatus != RobotCargoStatus::CARGO_LOADED)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting to unload %s", currentTaskKey.c_str());
            return;
        }
    }

    if (!currentTask.m_isDummy && currentTask.m_path) {
        if (m_robotStatus.m_finishedNavigationTask != currentTaskKey)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigating to %s", currentTaskKey.c_str());
            // Waiting for navigation
            return;
        }
        if (m_robotStatus.m_currentNavigationTask!= currentTaskKey)
        {
            m_robotStatus.m_finishedNavigationTask = "";
            m_nav2ActionClient.SendGoal(
                    currentTask.m_path.value(),
                    std::bind(&OttoAutonomy::NavigationGoalCompleted, this, std::placeholders::_1, currentTaskKey),
                    currentTask.m_isBlind, currentTask.m_isReverse);
        }
    }

    m_robotStatus.m_currentTask = currentTask.m_nextTaskName;

}

void OttoAutonomy::NavigationGoalCompleted(bool success, const RobotTaskKey& taskName)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Navigating finished  %s", taskName.c_str());
    if (success)
    {
        m_robotStatus.m_finishedNavigationTask = taskName;
    }
    else
    {
        // TODO - how to recover?
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to navigate, task name %s", taskName.c_str());
    }
}

void OttoAutonomy::NotifyCargoChanged(bool hasCargoNow)
{
    m_robotStatus.m_cargoStatus = hasCargoNow ? RobotCargoStatus::CARGO_LOADED : RobotCargoStatus::CARGO_EMPTY;
}
