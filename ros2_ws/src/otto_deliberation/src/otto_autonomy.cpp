
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
    if (!m_lockServiceClient->wait_for_service(std::chrono::seconds(30)))
    {
        RCLCPP_ERROR(lock_node->get_logger(), "Lock service named %s not available", lock_service.c_str());
    }

    m_lifterPublisher = node->create_publisher<std_msgs::msg::Bool>("lifter", 10);
    m_colorPublisher = node->create_publisher<std_msgs::msg::String>("color", 10);
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
    static const std::set<std::string> lanePaths = { "GoToPickup", "GoToWrapping" };

    auto name = path_name;
    if (lanePaths.count(path_name) > 0)
    {
        name += m_laneName;
    }

    auto lockRequest = std::make_shared<lock_service_msgs::srv::Lock::Request>();
    lockRequest->key = name;
    lockRequest->lock_status = lock_status;
    auto future = m_lockServiceClient->async_send_request(lockRequest);
    future.wait();
    auto result = future.get();
    return result->result;
}
void OttoAutonomy::SendColor (const std::string& color)
{
    auto msg = std_msgs::msg::String();
    msg.data = color;
    m_colorPublisher->publish(msg);
}

void OttoAutonomy::Update()
{
    if (m_robotTasks.GetTasks().empty())
    {
        return;
    }
    m_currentOperationDescription = "";
    const auto currentTaskKey = m_robotStatus.m_currentTask;
    const auto& currentTask = m_robotTasks.ConstructTask(currentTaskKey);

    // wait for pre-task delay
    if (currentTask.m_preTaskDelay)
    {
        if (!m_isWaitingPreTaskDelay)
        {
            m_isWaitingPreTaskDelay = true;
            m_waitTimePointPreTaskDelay = std::chrono::system_clock::now();
        }
        // todo use ros timer instead of chrono one
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_waitTimePointPreTaskDelay);
        if (duration.count() < currentTask.m_preTaskDelay.value())
        {
            // Waiting for pre-task delay
            m_currentOperationDescription += "Waiting to predelay at " + currentTaskKey + "\n";
            SendColor("YELLOW");
            return;
        }
    }

    // try to lock
    if (!m_hasLock && currentTask.m_isAcquiresLock)
    {
        if (!SendLockRequest(currentTask.m_taskName, true))
        {
            m_currentOperationDescription += "Waiting to lock at " + currentTaskKey + "\n";
            auto steady_clock = rclcpp::Clock();
            RCLCPP_ERROR_THROTTLE(m_logger,steady_clock, 1500, "WAITING FOR LOCK");
            SendColor("BLUE");
            return;
        }
        m_currentOperationDescription += "Lock obtained " + currentTaskKey + "\n";
        m_lockTaskName = currentTask.m_taskName;
        RCLCPP_INFO(m_logger, "Lock obtained %s", m_lockTaskName.c_str());
        m_hasLock = true;
    }

    std_msgs::msg::Bool lifterStatus;
    lifterStatus.data = currentTask.m_isLifter;
    m_lifterPublisher->publish(lifterStatus);

    if (currentTask.m_isCargoLoad)
    {
        if (m_robotStatus.m_cargoStatus != RobotCargoStatus::CARGO_LOADED)
        {
            m_currentOperationDescription += "Waiting to load at " + currentTaskKey + "\n";
            SendColor("ORANGE");
            return;
        }
    }

    if (currentTask.m_isCargoUnload)
    {
        if (m_robotStatus.m_cargoStatus != RobotCargoStatus::CARGO_EMPTY)
        {
            m_currentOperationDescription += "Waiting to unload at " + currentTaskKey + "\n";
            SendColor("GREEN");
            return;
        }
    }

    if (!currentTask.m_isDummy && !currentTask.m_path->poses.empty())
    {
        if (m_robotStatus.m_currentNavigationTask != currentTaskKey || m_robotStatus.m_resendGoal)
        {
            if (m_robotStatus.m_resendGoal)
            {
                RCLCPP_ERROR(m_logger, "!!!!!! Resending goal for task %s", currentTaskKey.c_str());
                m_robotStatus.m_resendGoal = false;
            }
            m_startNavigationTimePoint = std::chrono::system_clock::now();
            m_currentOperationDescription += "Started navigation at " + currentTaskKey + "\n";
            m_robotStatus.m_currentNavigationTask = currentTaskKey;
            m_robotStatus.m_finishedNavigationTask = "";
            m_nav2ActionClient.SendGoal(
                *currentTask.m_path,
                std::bind(&OttoAutonomy::NavigationGoalCompleted, this, std::placeholders::_1),
                currentTask.m_isBlind,
                currentTask.m_isReverse,
                currentTask.m_isBlindHighSpeed);
            RCLCPP_ERROR(m_logger, "Sending goal for task %s", currentTaskKey.c_str());
            SendColor("");
        }
        if (m_robotStatus.m_finishedNavigationTask != currentTaskKey)
        {

            auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_startNavigationTimePoint);
            m_currentOperationDescription += "Navigating to " + currentTaskKey + " is blind : " + std::to_string(currentTask.m_isBlind) +" elapsed time : " + std::to_string(duration.count()) + "\n";
            return;
        }
    }

    // wait for post-task delay
    if (currentTask.m_postTaskDelay)
    {
        if (!m_isWaitingPostTaskDelay)
        {
            m_isWaitingPostTaskDelay = true;
            m_waitTimePointPostTaskDelay = std::chrono::system_clock::now();
        }
        // todo use ros timer instead of chrono one
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - m_waitTimePointPostTaskDelay);
        if (duration.count() < currentTask.m_postTaskDelay.value())
        {
            // Waiting for pre-task delay
            m_currentOperationDescription += "Waiting post delay at " + currentTaskKey + "\n";
            return;
        }
    }

    // unlock
    if (currentTask.m_isReleasesLock && m_hasLock)
    {
        // check if next task has lock, if so try lock it to prevent releasing this task
        const auto& nextTaskName = currentTask.m_nextTaskName;
        if (!nextTaskName.empty() && m_robotTasks.GetIfTaskNeedsLock(nextTaskName))
        {
            if (!SendLockRequest(nextTaskName, true))
            {
                m_currentOperationDescription += "Waiting to acquire next task lock " + nextTaskName + " at " + currentTaskKey + "\n";
                return;
            }
            // release old lock
            SendLockRequest(m_lockTaskName, false);
            m_currentOperationDescription += "Lock obtained " + nextTaskName + ", releasing lock " + m_lockTaskName + "\n";
            RCLCPP_ERROR(m_logger, "Lock obtained %s, releasing lock %s", nextTaskName.c_str(), m_lockTaskName.c_str());
            m_lockTaskName = nextTaskName;
            m_hasLock = true;
        }
        else
        {
            SendLockRequest(m_lockTaskName, false);
            m_currentOperationDescription += "Releasing lock " + m_lockTaskName + "\n";
            RCLCPP_ERROR(m_logger, "Releasing lock %s", m_lockTaskName.c_str());
            m_lockTaskName = "";
            m_hasLock = false;
        }
    }

    if (currentTask.m_nextTaskName.empty())
    {
        return;
    }
    else
    {
        m_currentOperationDescription = "Task transition " + currentTaskKey + " -> " + currentTask.m_nextTaskName;
        RCLCPP_ERROR(m_logger, "task transition from  %s - > %s", currentTaskKey.c_str(), currentTask.m_nextTaskName.c_str());
    }

    m_robotStatus.m_currentTask = currentTask.m_nextTaskName;
    m_isWaitingPreTaskDelay = false;
    m_isWaitingPostTaskDelay = false;
    m_robotStatus.m_resendGoal = false;
}

void OttoAutonomy::NavigationGoalCompleted(bool success)
{
    RCLCPP_INFO(m_logger, "Navigating finished  %s", m_robotStatus.m_currentNavigationTask.c_str());
    if (success)
    {
        m_robotStatus.m_finishedNavigationTask = m_robotStatus.m_currentNavigationTask;
    }
    else
    {
        // TODO - how to recover?
        RCLCPP_ERROR(m_logger, "Failed to navigate, task name %s", m_robotStatus.m_currentNavigationTask.c_str());
        m_robotStatus.m_resendGoal = true;

    }
}

void OttoAutonomy::NotifyCargoChanged(bool hasCargoNow)
{
    m_robotStatus.m_cargoStatus = hasCargoNow ? RobotCargoStatus::CARGO_LOADED : RobotCargoStatus::CARGO_EMPTY;
}

std::string OttoAutonomy::GetCurrentOperationDescription() const
{
    return m_currentOperationDescription;
}

std::string OttoAutonomy::GetCurrentTaskName() const
{
    return m_robotStatus.m_currentTask;
}
