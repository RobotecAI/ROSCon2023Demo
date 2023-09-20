
#include <otto_deliberation/otto_autonomy.h>

OttoAutonomy::OttoAutonomy(rclcpp::Node::SharedPtr node) : m_logger(node->get_logger()), m_nav2ActionClient(node) {}

void OttoAutonomy::SetTasks(const Tasks& tasks, bool loop) {
	m_loop = loop;
	m_tasks = tasks;
}

RobotStatus OttoAutonomy::GetCurrentStatus() const {
	return m_robotStatus;
}

void OttoAutonomy::Update() {
	if (m_tasks.empty()) {
		return;
	}

	auto currentTask = m_tasks.front();
	if (currentTask.m_goalTaskStatus == m_robotStatus.m_taskStatus) {
		if (currentTask.m_requiredCargoStatus != m_robotStatus.m_cargoStatus) {
			// Waiting for load/unload
			return;
		}
		// Current task completed!
		if (m_loop) {
			m_tasks.push(currentTask);
		}
		m_tasks.pop();

		// Go to next task if any
		if (m_tasks.empty()) {
			return;
		}

		auto nextTask = m_tasks.front();
		m_robotStatus.m_currentTaskKey = nextTask.m_taskKey;
		if (nextTask.m_path.empty()) {
			// Trivially achieved navigation goal, but might need cargo status change to achieve task completion.
			NavigationGoalCompleted(true);
		} else {
			m_nav2ActionClient.SendGoal(nextTask.m_path,
			                            std::bind(&OttoAutonomy::NavigationGoalCompleted, this, std::placeholders::_1),
			                            TaskUtils::IsTaskBlind(nextTask.m_taskKey), nextTask.m_reverse);
		}
	}
}

void OttoAutonomy::NavigationGoalCompleted(bool success) {
	if (success) {
		m_robotStatus.m_taskStatus = m_tasks.front().m_goalTaskStatus;
	}
	// TODO - how to recover?
}

void OttoAutonomy::NotifyCargoChanged(bool hasCargoNow) {
	m_robotStatus.m_cargoStatus = hasCargoNow ? RobotCargoStatus::CARGO_LOADED : RobotCargoStatus::CARGO_EMPTY;
}
