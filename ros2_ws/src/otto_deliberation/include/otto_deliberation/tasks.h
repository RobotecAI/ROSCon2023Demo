
#include <geometry_msgs/msg/pose_stamped.h>
#include <map>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <otto_deliberation/robot_status.h>
#include <queue>
#include <string>
#include <string_view>

#pragma once

typedef nav_msgs::msg::Path NavPath;

struct Task
{
    std::string m_taskKey; // unique task/path name
    bool m_requiresLock; // Whether the task needs a lock to start (and releases it when complete).
    bool m_reverse; // Whether the task is to be done driving backwards
    NavPath m_path;
    RobotCargoStatus m_requiredCargoStatus; // wait for this cargo status before completing.
    RobotTaskStatus m_goalTaskStatus; // change to this status once task is completed.
};

typedef std::deque<Task> Tasks;
typedef std::map<std::string, NavPath> NamedPathsMap;

class TaskUtils
{
public:
    static bool IsTaskBlind(const std::string& taskKey)
    {
        static const std::set<std::string> blindTasks = {
            "ApproachPickup", "EvacuateFromPickup", "DoWrapping", "ApproachUnload", "EvacuateFromUnload"
        };
        return blindTasks.count(taskKey) == 1;
    }

    static RobotCargoStatus GetCargoStatus(const std::string& taskKey)
    {
        static const std::set<std::string> cargoTasks = { "WaitLoad",   "EvacuateFromPickup", "GoToWrapping",  "DoWrapping",
                                                          "GoToUnload", "ApproachUnload",     "ApproachPickup" };
        return cargoTasks.count(taskKey) == 1 ? RobotCargoStatus::CARGO_LOADED : RobotCargoStatus::CARGO_EMPTY;
    }

    static RobotTaskStatus GetTaskStatus(const std::string& taskKey)
    {
        static const std::map<std::string, RobotTaskStatus> taskStatuses = {
            { "Idle", RobotTaskStatus::IDLE },
            { "GoToPickup", RobotTaskStatus::GO_TO_PICK_UP },
            { "ApproachPickup", RobotTaskStatus::APPROACH_PICKUP },
            { "WaitLoad", RobotTaskStatus::WAIT_LOAD },
            { "EvacuateFromPickup", RobotTaskStatus::EVACUATE_FROM_PICK_UP },
            { "GoToWrapping", RobotTaskStatus::GO_TO_WRAPPING },
            { "DoWrapping", RobotTaskStatus::DO_WRAPPING },
            { "GoToUnload", RobotTaskStatus::GO_TO_UNLOAD },
            { "ApproachUnload", RobotTaskStatus::APPROACH_UNLOAD },
            { "WaitUnload", RobotTaskStatus::WAIT_LOAD },
            { "EvacuateFromUnload", RobotTaskStatus::EVACUATE_FROM_UNLOAD },
        };
        if (taskStatuses.find(taskKey) == taskStatuses.end())
        {
            return RobotTaskStatus::IDLE;
        }
        return taskStatuses.find(taskKey)->second;
    }

    static bool GetReverse(const std::string& taskKey)
    {
        static constexpr std::string_view key("Evacuate");
        return taskKey.find(key) != std::string::npos ? true : false;
    }
};
