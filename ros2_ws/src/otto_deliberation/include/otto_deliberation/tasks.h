#pragma once

#include <geometry_msgs/msg/pose_stamped.h>
#include <nav_msgs/msg/detail/path__struct.hpp>

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

using NavPath = nav_msgs::msg::Path;
using NavPathPtr = std::shared_ptr<NavPath>;
using RobotTaskKey = std::string;
using RobotTaskList = std::vector<RobotTaskKey>;
using RobotTaskSet = std::unordered_set<RobotTaskKey>;

//! Task definition for robot
struct Task
{
    RobotTaskKey m_taskName;
    RobotTaskKey m_nextTaskName;
    bool m_isBlind;
    bool m_isReverse;
    bool m_isLifter;
    bool m_isDummy;
    bool m_isAcquiresLock;
    bool m_isReleasesLock;
    bool m_isCargoUnload;
    bool m_isCargoLoad;
    bool m_isBlindHighSpeed;
    std::optional<double> m_preTaskDelay;
    std::optional<double> m_postTaskDelay;
    NavPathPtr m_path;
};

//! Robot tasks container class
class RobotTasks
{
private:
    mutable RobotTaskSet m_validTasks; //!< Tasks that are valid, accelerates validation
    mutable std::unordered_map<RobotTaskKey, RobotTaskKey> m_taskTransitions; //!< Map of task transitions, accelerates transition lookup

public:
    bool m_loop = false; //!< Whether to loop the tasks
    RobotTaskList m_tasks; //!< Complete list of tasks
    RobotTaskSet m_lifterTasks; //!< Tasks that need lifter in lifted position
    RobotTaskSet m_dummyTasks; //!< Tasks that has robot in dummy mode
    RobotTaskSet m_blindTasks; //!< Tasks that are blind (use blind path follower instead nav stack)
    RobotTaskSet m_blindTasksReverse; //!< Tasks that are blind and need to be reversed
    RobotTaskSet m_cargoLoadTasks; //!< Tasks that need to have cargo m_taskPaths to continue
    RobotTaskSet m_cargoUnLoadTasks; //!< Tasks that need to have cargo un loaded to continue
    RobotTaskSet m_acquireLock; //!< Tasks that needs acquire a lock
    RobotTaskSet m_releaseLock; //!< Tasks that releases a lock
    RobotTaskSet m_blindHighSpeed; //!< Tasks that are high speed blind
    std::unordered_map<RobotTaskKey, double> m_postTaskDelays; //!< Tasks that need delay to start
    std::unordered_map<RobotTaskKey, double> m_preTaskDelay; //!< Tasks that need delay to finish
    std::unordered_map<RobotTaskKey, NavPathPtr> m_taskPaths; //!< Geometry of tasks

public:
    //! Print list of tasks
    void PrintTasks();

    //! Validate that all taks are defined correctly
    //! @return true if all tasks were defined correctly
    bool ValidateTasks() const;

    //! Get the list of tasks
    const RobotTaskList& GetTasks() const;

    //! Get if task is defined in the RobotTasks
    bool IsTaskValid(const RobotTaskKey& taskName) const;

    //! Get next task name
    //! @param taskName current task name
    //! @return next task name
    RobotTaskKey GetNextTaskName(const RobotTaskKey& taskName) const;

    //! Return true if task needs lock to be executed
    //! @param taskName current task name
    //! @return next task name
    bool GetIfTaskNeedsLock(const RobotTaskKey& taskName) const;

    //! Get task with a given name
    //! @param taskName task name
    //! @return task definition, emtpy task if not found
    Task ConstructTask(const RobotTaskKey& taskKey) const;
};
