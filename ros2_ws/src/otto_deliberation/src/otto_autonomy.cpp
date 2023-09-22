
#include "otto_deliberation/robot_status.h"
#include "otto_deliberation/tasks.h"
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

void OttoAutonomy::SetTasks(const Tasks& tasks, bool loop)
{
    m_loop = loop;
    m_tasks = tasks;
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
    if (m_tasks.empty())
    {
        return;
    }

    auto currentTask = m_tasks.front();
    if (currentTask.m_goalTaskStatus == m_robotStatus.m_taskStatus)
    {
        if (currentTask.m_requiredCargoStatus != m_robotStatus.m_cargoStatus)
        {
            // Waiting for load/unload
            return;
        }

        if (m_tasks.size() > 1)
        {
            if (m_tasks[1].m_requiresLock)
            {
                if (!SendLockRequest(m_tasks[1].m_taskKey, true))
                {
                    return;
                }
            }
        }
        if (currentTask.m_requiresLock)
        {
            if (!SendLockRequest(currentTask.m_taskKey, false))
            {
                return;
            }
        }

        // Current task completed!
        if (m_loop)
        {
            m_tasks.push_back(currentTask);
        }
        m_tasks.pop_front();

        // Go to next task if any
        if (m_tasks.empty())
        {
            return;
        }

        auto nextTask = m_tasks.front();
        m_robotStatus.m_currentTaskKey = nextTask.m_taskKey;
        if (nextTask.m_path.poses.empty())
        {
            // Trivially achieved navigation goal, but might need cargo status change to achieve task completion.
            NavigationGoalCompleted(true);
        }
        else
        {
            m_nav2ActionClient.SendGoal(
                nextTask.m_path,
                std::bind(&OttoAutonomy::NavigationGoalCompleted, this, std::placeholders::_1),
                TaskUtils::IsTaskBlind(nextTask.m_taskKey),
                nextTask.m_reverse);
        }
        std_msgs::msg::Bool lifterStatus;
        lifterStatus.data = nextTask.m_lifterUp;
        m_lifterPublisher->publish(lifterStatus);
    }
}

void OttoAutonomy::NavigationGoalCompleted(bool success)
{
    if (success)
    {
        m_robotStatus.m_taskStatus = m_tasks.front().m_goalTaskStatus;
    }
    // TODO - how to recover?
}

void OttoAutonomy::NotifyCargoChanged(bool hasCargoNow)
{
    m_robotStatus.m_cargoStatus = hasCargoNow ? RobotCargoStatus::CARGO_LOADED : RobotCargoStatus::CARGO_EMPTY;
}
