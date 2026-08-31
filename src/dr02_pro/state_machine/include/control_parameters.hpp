/**
 * @file control_parameters.hpp
 * @brief Control parameter definitions.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <iostream>

#include "types.h"

namespace deep_robotics::dr02_pro {

class ControlParameters {
 public:
  /**
   * @brief Construct control parameters for a robot.
   * @param robot_name Robot name.
   */
  explicit ControlParameters(RobotName robot_name) {
    if (robot_name != RobotName::DR2PRO) {
      std::cerr << "Not Default Robot" << std::endl;
      return;
    }
    dof_num_ = 31;
    arm_kp_ = VecXf::Zero(7);
    arm_kd_ = VecXf::Zero(7);
    waist_kp_ = VecXf::Zero(3);
    waist_kd_ = VecXf::Zero(3);

    waist_kp_ << 600., 2800., 2300.;
    waist_kd_ << 6., 15., 20.;
    arm_kp_ << 600., 600., 600., 600., 90., 90., 90.;
    arm_kd_ << 6., 6., 6., 6., 2., 2., 2.;
    leg_kp_ << 1800.0, 600.0, 600.0, 1800.0, 600.0, 90.0;
    leg_kd_ << 24.0, 6.0, 6.0, 24.0, 6.0, 2.0;
    neck_kp_ << 0., 0.;
    neck_kd_ << 0., 0.;
  }

  int dof_num_{0};
  VecXf arm_kp_;
  VecXf arm_kd_;
  Vec6f leg_kp_{};
  Vec6f leg_kd_{};
  VecXf waist_kp_;
  VecXf waist_kd_;
  Vec2f neck_kp_{};
  Vec2f neck_kd_{};
};

}  // namespace deep_robotics::dr02_pro
