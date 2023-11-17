#pragma once

#include "tasks.h"

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
    RobotCargoStatus m_cargoStatus{ RobotCargoStatus::CARGO_EMPTY };
    bool m_resendGoal{ false };
    RobotStatus() = default;
};
