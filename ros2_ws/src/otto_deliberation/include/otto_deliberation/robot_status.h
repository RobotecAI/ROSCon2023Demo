
#include <rclcpp/rclcpp.hpp>

#pragma once

enum class RobotCargoStatus
{
	CARGO_EMPTY,
	CARGO_LOADED
};

enum class RobotTaskStatus
{
	IDLE,
	GO_TO_PICK_UP,
	APPROACH_PICKUP,
	WAIT_LOAD,
	EVACUATE_FROM_PICK_UP,
	GO_TO_WRAPPING,
	GO_TO_WRAPPING_GLOBAL,
	DO_WRAPPING,
	GO_TO_UNLOAD,
	APPROACH_UNLOAD,
	WAIT_UNLOAD,
	EVACUATE_FROM_UNLOAD
};

struct RobotStatus
{
	RobotTaskStatus m_taskStatus;
	RobotCargoStatus m_cargoStatus;
	std::string m_currentTaskKey;
	RobotStatus() {
		m_cargoStatus = RobotCargoStatus::CARGO_EMPTY;
		m_taskStatus = RobotTaskStatus::IDLE;
	}
};
