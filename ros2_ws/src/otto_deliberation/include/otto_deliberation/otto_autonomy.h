
#include <chrono>
#include <lock_service_msgs/srv/detail/lock__struct.hpp>
#include <otto_deliberation/nav2_action_client.h>
#include <otto_deliberation/robot_status.h>
#include <otto_deliberation/tasks.h>
#include <rclcpp/client.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#pragma once

class OttoAutonomy
{
public:
	OttoAutonomy(rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr lock_node);
	void SetTasks(const Tasks& tasks, bool loop = true);
	void SetLane(const std::string& lane_name);
	void Update();
	void NavigationGoalCompleted(bool success);
	void NotifyCargoChanged(bool hasCargoNow);
	RobotStatus GetCurrentStatus() const;

private:
	bool SendLockRequest(const std::string& path_name, bool lock_status);

	rclcpp::Logger m_logger;
	rclcpp::Client<lock_service_msgs::srv::Lock>::SharedPtr m_lockServiceClient;
	RobotStatus m_robotStatus;
	Nav2ActionClient m_nav2ActionClient;
	Tasks m_tasks;  // front task is the current one.
	std::string m_laneName;
	std::chrono::time_point<std::chrono::system_clock> m_waitTimePoint;
	bool m_isWaiting;
	bool m_loop;
	bool m_hasLock;
};
