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

#include <cstdint>

#include "common_types.hpp"

namespace deep_robotics::dr02_pro {

using namespace deep_robotics::common;

struct RobotBasicState {
  VecXf joint_pos;
  VecXf joint_vel;
  MatXf flt_joint_vel_mat;
  VecXf joint_tau;
};

struct RobotAction {
  VecXf goal_joint_pos;
  VecXf goal_joint_vel;
  VecXf kp;
  VecXf kd;
  VecXf tau_ff;

  /**
   * @brief Convert the action fields into a joint command matrix.
   * @return Joint command matrix.
   */
  MatXf ConvertToMat() {
    MatXf res(goal_joint_pos.rows(), 5);
    res.col(0) = kp;
    res.col(1) = goal_joint_pos;
    res.col(2) = kd;
    res.col(3) = goal_joint_vel;
    res.col(4) = tau_ff;
    return res;
  }
};

struct UserCommand {
  double time_stamp;
  int safe_control_mode;
  uint8_t target_mode;
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
    kInvalid = -1,
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

enum CommandIndex {
  kIndexDisable = 1,
  kIndexEnable = 2,
  kIndexMotorControl = 4,
  kIndexErrorReset = 17
};

}  // namespace deep_robotics::dr02_pro
