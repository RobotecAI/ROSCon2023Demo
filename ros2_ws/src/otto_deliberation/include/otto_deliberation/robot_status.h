
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#pragma once

enum class RobotCargoStatus
{
	CARGO_EMPTY,
	CARGO_LOADED
};

using  RobotTaskKey = std::string;
constexpr std::string_view TASK_IDLE = "Idle";
using RobotTaskList = std::vector<RobotTaskKey>;
using RobotTaskSet = std::unordered_set<RobotTaskKey>;

struct Task
{
    RobotTaskKey m_taskName;
    RobotTaskKey m_nextTaskName;
    bool m_isBlind;
    bool m_isReverse;
    bool m_isLifter;
    bool m_isDummy;
    bool m_isNeedLock;
    bool m_isCargoUnload;
    bool m_isCargoLoad;
    std::optional<double> m_preTaskDelay;
    std::optional<double> m_postTaskDelay;
    std::optional<nav_msgs::msg::Path> m_path;
};

struct RobotTasks
{
    bool m_loop = false; //!< Whether to loop the tasks
    RobotTaskList m_tasks ; //!< Complete list of tasks
    RobotTaskSet m_lifterTasks; //!< Tasks that need lifter in lifted position
    RobotTaskSet m_dummyTasks; //!< Tasks that has robot in dummy mode
    RobotTaskSet m_blindTasks; //!< Tasks that are blind (use blind path follower instead nav stack)
    RobotTaskSet m_blindTasksReverse; //!< Tasks that are blind and need to be reversed
    RobotTaskSet m_cargoLoadTasks; //!< Tasks that need to have cargo loaded to continue
    RobotTaskSet m_cargoUnLoadTasks; //!< Tasks that need to have cargo un loaded to continue
    RobotTaskSet m_tasksWithLock; //!< Tasks that need lock to start
    std::unordered_map<RobotTaskKey, double> m_postTaskDelays; //!< Tasks that need delay to start
    std::unordered_map<RobotTaskKey, double> m_preTaskDelay; //!< Tasks that need delay to finish
    std::unordered_map<RobotTaskKey,nav_msgs::msg::Path> m_taskPaths; //!< Geometry of tasks

    void PrintTasks()
    {
        for (auto & m : m_tasks)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Task %s", m.c_str());
        }
    }
    //! Validate that all taks are defined correctly
    void ValidateTasks() const
    {
        std::unordered_set<RobotTaskKey> tasksSet(m_tasks.begin(), m_tasks.end());

        for (auto & m : m_lifterTasks)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in lifter tasks but not in tasks list", m.c_str());
            }
        }

        for (auto & m : m_dummyTasks)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in dummy tasks but not in tasks list", m.c_str());
            }
        }

        for (auto & m : m_blindTasks)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in blind tasks but not in tasks list", m.c_str());
            }
        }

        for (auto & m : m_blindTasksReverse)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in blind tasks reverse but not in tasks list", m.c_str());
            }
        }

        for (auto & m : m_cargoUnLoadTasks)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in cargo unload but not in tasks list", m.c_str());
            }
        }

        for (auto & m : m_tasksWithLock)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in task with lock but not in tasks list", m.c_str());
            }
        }

        for (auto & [m,_] : m_postTaskDelays)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in task with pre-delay but not in tasks list", m.c_str());
            }
        }

        for (auto & [m,_] : m_preTaskDelay)
        {
            if (tasksSet.count(m) == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is in task with post-delay but not in tasks list", m.c_str());
            }
        }

        for (const auto &t : tasksSet)
        {
            if (m_dummyTasks.count(t)== 0 && m_taskPaths.count(t)== 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Task %s is not in dummy tasks but has no valid geomtry paths", t.c_str());
            }
        }

    }

    //! Get the list of tasks
    const RobotTaskList& GetTasks() const
    {
        return m_tasks;
    }

    bool IsTaskValid(const RobotTaskKey& taskName) const
    {
        return std::find(m_tasks.begin(), m_tasks.end(), taskName) != m_tasks.end();
    }

    //! Get next task name
    RobotTaskKey GetNextTaskName(const RobotTaskKey& taskName) const
    {
        auto it = std::find(m_tasks.begin(), m_tasks.end(), taskName);
        if (it == m_tasks.end())
        {
            return "";
        }
        it++;
        if (it == m_tasks.end() && m_loop)
        {
            it = m_tasks.begin();
        }
        else if (it == m_tasks.end())
        {
            return "";
        }
        return *it;
    }

    Task ConstructTask(const RobotTaskKey& taskKey)
    {
        if (IsTaskValid(taskKey))
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
        task.m_isNeedLock = m_tasksWithLock.count(taskKey) > 0;
        task.m_isCargoUnload = m_cargoUnLoadTasks.count(taskKey) > 0;
        task.m_isCargoLoad = m_cargoLoadTasks.count(taskKey) > 0;
        if (m_taskPaths.count(taskKey) > 0)
        {
            task.m_path = m_taskPaths[taskKey];
        }
        if(m_preTaskDelay.count(taskKey) > 0)
        {
            task.m_preTaskDelay = m_preTaskDelay[taskKey];
        }
        if(m_postTaskDelays.count(taskKey) > 0)
        {
            task.m_postTaskDelay = m_postTaskDelays[taskKey];
        }
    }

};



struct RobotStatus
{
    RobotTaskKey m_currentTask;
    RobotTaskKey m_currentNavigationTask;
    RobotTaskKey m_finishedNavigationTask;
	RobotCargoStatus m_cargoStatus;
	RobotStatus() {
		m_cargoStatus = RobotCargoStatus::CARGO_EMPTY;
	}
};
