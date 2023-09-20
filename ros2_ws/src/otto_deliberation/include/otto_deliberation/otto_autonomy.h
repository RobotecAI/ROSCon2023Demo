
#include <otto_deliberation/nav2_action_client.h>
#include <otto_deliberation/robot_status.h>
#include <otto_deliberation/tasks.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#pragma once

class OttoAutonomy
{
public:
	OttoAutonomy(rclcpp::Node::SharedPtr node);
	void SetTasks(const Tasks& tasks, bool loop = true);
	void Update();
	void NavigationGoalCompleted(bool success);
	void NotifyCargoChanged(bool hasCargoNow);
	RobotStatus GetCurrentStatus() const;

private:
	rclcpp::Logger m_logger;
	RobotStatus m_robotStatus;
	Nav2ActionClient m_nav2ActionClient;
	Tasks m_tasks;  // front task is the current one.
	bool m_loop;
};
