#include "otto_deliberation/tasks.h"

#include <rclcpp/rclcpp.hpp>

void RobotTasks::PrintTasks() const
{
    for (auto& m : m_tasks)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task %s", m.c_str());
    }
}

bool RobotTasks::ValidateTasks() const
{
    bool isValid = true;
    m_validTasks = RobotTaskSet(m_tasks.begin(), m_tasks.end());

    isValid = ValidateTaskSet(m_lifterTasks, "lifter tasks") && isValid;
    isValid = ValidateTaskSet(m_dummyTasks, "dummy tasks") && isValid;
    isValid = ValidateTaskSet(m_blindTasks, "blind tasks") && isValid;
    isValid = ValidateTaskSet(m_blindTasksReverse, "blind tasks reverse") && isValid;
    isValid = ValidateTaskSet(m_blindHighSpeed, "blind tasks high speed") && isValid;
    isValid = ValidateTaskSet(m_cargoUnLoadTasks, "cargo unload") && isValid;
    isValid = ValidateTaskSet(m_acquireLock, "tasks with lock (acquire)") && isValid;
    isValid = ValidateTaskSet(m_releaseLock, "tasks with lock (release)") && isValid;

    isValid = ValidateTaskDelays() && isValid;
    isValid = ValidateTaskPaths() && isValid;

    if (m_validTasks.empty())
    {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Technically, empty scenario is valid, but it is not very useful.");
    }

    bool lock = false;
    for (auto &taskName : m_tasks)
    {
        const auto task = ConstructTask(taskName);
        if (task.m_isAcquiresLock)
        {
            lock = true;
        }
        if (task.m_isReleasesLock)
        {
            lock = false;
        }
    }
    if (lock)
    {
            isValid = false;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Lock is not released at the end of the scenario");
    }
    return isValid;
}

const RobotTaskList& RobotTasks::GetTasks() const
{
    return m_tasks;
}

bool RobotTasks::IsTaskValid(const RobotTaskKey& taskName) const
{
    if (m_validTasks.empty())
    {
        m_validTasks = RobotTaskSet(m_tasks.begin(), m_tasks.end());
    }
    return m_validTasks.count(taskName) > 0;
}

RobotTaskKey RobotTasks::GetNextTaskName(const RobotTaskKey& taskName) const
{
    if (!IsTaskValid(taskName))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid task name %s", taskName.c_str());
        return "";
    }

    if (m_taskTransitions.empty())
    {
        // construct transitions
        for (size_t i = 0; i < m_tasks.size(); i++)
        {
            const auto currentTaskName = m_tasks[i];
            if (i + 1 >= m_tasks.size())
            {
                if (m_loop)
                {
                    m_taskTransitions[currentTaskName] = m_tasks[0];
                }
                else
                {
                    m_taskTransitions[currentTaskName] = "";
                }
            }
            else
            {
                m_taskTransitions[currentTaskName] = m_tasks[i + 1];
            }
        }
    }
    const auto it = m_taskTransitions.find(taskName);

    if (it == m_taskTransitions.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s has no next task", taskName.c_str());
        return "";
    }
    return it->second;
}

Task RobotTasks::ConstructTask(const RobotTaskKey& taskKey) const
{
    if (!IsTaskValid(taskKey))
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid task name %s", taskKey.c_str());
        return Task();
    }
    Task task;
    task.m_taskName = taskKey;
    task.m_nextTaskName = GetNextTaskName(taskKey);
    task.m_isBlind = m_blindTasks.count(taskKey) > 0;
    task.m_isReverse = m_blindTasksReverse.count(taskKey) > 0;
    task.m_isLifter = m_lifterTasks.count(taskKey) > 0;
    task.m_isDummy = m_dummyTasks.count(taskKey) > 0;
    task.m_isAcquiresLock = m_acquireLock.count(taskKey) > 0;
    task.m_isReleasesLock = m_releaseLock.count(taskKey) > 0;
    task.m_isCargoUnload = m_cargoUnLoadTasks.count(taskKey) > 0;
    task.m_isCargoLoad = m_cargoLoadTasks.count(taskKey) > 0;
    task.m_isBlindHighSpeed = m_blindHighSpeed.count(taskKey) > 0;
    if (m_taskPaths.count(taskKey) > 0)
    {
        task.m_path = m_taskPaths.at(taskKey);
    }
    if (m_preTaskDelay.count(taskKey) > 0)
    {
        task.m_preTaskDelay = m_preTaskDelay.at(taskKey);
    }
    if (m_postTaskDelays.count(taskKey) > 0)
    {
        task.m_postTaskDelay = m_postTaskDelays.at(taskKey);
    }
    return task;
}

bool RobotTasks::GetIfTaskNeedsLock(const RobotTaskKey& taskName) const
{
    return m_acquireLock.count(taskName) > 0;
}

bool RobotTasks::ValidateTaskSet(const RobotTaskSet& taskSet, const std::string& taskSetName) const
{
    bool isValid = true;
    for (const auto& task : taskSet)
    {
        if (m_validTasks.count(task) == 0)
        {
            isValid = false;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in %s but not in tasks list", task.c_str(), taskSetName.c_str());
        }
    }
    return isValid;
}

bool RobotTasks::ValidateTaskDelays() const
{
    bool isValid = true;

    for (auto& [m, _] : m_postTaskDelays)
    {
        if (m_validTasks.count(m) == 0)
        {
            isValid = false;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in task with post-delay but not in tasks list", m.c_str());
        }
    }

    for (auto& [m, _] : m_preTaskDelay)
    {
        if (m_validTasks.count(m) == 0)
        {
            isValid = false;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in task with pre-delay but not in tasks list", m.c_str());
        }
    }

    return isValid;
}

bool RobotTasks::ValidateTaskPaths() const
{
    bool isValid = true;

    for (const auto& t : m_validTasks)
    {
        if (m_dummyTasks.count(t) == 0 && m_taskPaths.count(t) == 0)
        {
            isValid = false;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is not in dummy tasks but has no valid geometry paths", t.c_str());
        }
    }

    for (const auto& t : m_validTasks)
    {
        if ( m_taskPaths.count(t)!= 0 &&  m_taskPaths.at(t) == nullptr)
        {
            isValid = false;
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is not in dummy tasks but has null geometry paths", t.c_str());
        }

    }

    return isValid;
}
