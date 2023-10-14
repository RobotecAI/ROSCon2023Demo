
#include "tasks.h"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#pragma once

enum class RobotCargoStatus
{
    CARGO_EMPTY,
    CARGO_LOADED
};

struct RobotStatus
{
    RobotTaskKey m_currentTask;
    RobotTaskKey m_currentNavigationTask;
    RobotTaskKey m_finishedNavigationTask;
    RobotCargoStatus m_cargoStatus;
    bool m_resendGoal{false};
    RobotStatus();
};
