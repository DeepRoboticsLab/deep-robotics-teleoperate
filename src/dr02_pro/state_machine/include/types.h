/**
 * @file types.h
 * @brief Shared state-machine data types.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <atomic>
#include <cstdint>

#include "common_types.hpp"

namespace deep_robotics::dr02_pro {

using namespace deep_robotics::common;

struct UserCommand {
  std::atomic<int> safe_control_mode{0};
  uint8_t target_mode{0};
};

enum RobotName {
    DR2PRO = 0,
};

enum RobotMotionState {
    Emergency = 0,
    ZeroPos = 1,
    TeleoperationMode = 2,
    DownMode = 3,
};

enum StateName {
    kEmergency = RobotMotionState::Emergency,
    kZeroPos = RobotMotionState::ZeroPos,
    kTeleoperation = RobotMotionState::TeleoperationMode,
    kDown = RobotMotionState::DownMode,
};

enum KeyCode {
    L1,
    L2,
    R1,
    R2,
    UNKNOWN
};

}  // namespace deep_robotics::dr02_pro
