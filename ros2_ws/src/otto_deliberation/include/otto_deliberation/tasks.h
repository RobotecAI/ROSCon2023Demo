
#include <geometry_msgs/msg/pose_stamped.h>
#include <otto_deliberation/robot_status.h>
#include <map>
#include <queue>
#include <string>

#pragma once

typedef std::vector<geometry_msgs::msg::PoseStamped> NavPath;

struct Task
{
	std::string m_taskKey;  // unique task/path name
	bool m_requiresLock;  // Whether the task needs a lock to start (and releases it when complete).
	NavPath m_path;
	RobotCargoStatus m_requiredCargoStatus;  // wait for this cargo status before completing.
	RobotTaskStatus m_goalTaskStatus;  // change to this status once task is completed.
};

typedef std::queue<Task> Tasks;
typedef std::map<std::string, NavPath> NamedPathsMap;

class TaskUtils
{
public:
	static bool IsTaskBlind(const std::string& taskKey) {
		static const std::set<std::string> blindTasks = { "ApproachPickup", "EvacuateFromPickUp", "DoWrapping",
			                                               "ApproachUnload", "EvacuateFromUnload" };
		return blindTasks.count(taskKey) == 1;
	}
};
