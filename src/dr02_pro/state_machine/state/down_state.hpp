/**
 * @file down_state.hpp
 * @brief Interpolated upper-body down-pose state.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <algorithm>
#include <array>

#include "state_base.h"

namespace deep_robotics::dr02_pro {

class DownState : public StateBase {
private:
    static constexpr int kUpperBodyDof = 17;
    static constexpr float kTransitionTime = 3.0f;

    float state_enter_time_{0.0f};
    VecXf joint_pos_init_;
    VecXf joint_vel_init_;
    VecXf joint_pos_goal_;
    VecXf joint_pos_hold_;
    VecXf kp_;
    VecXf kd_;
    MatXf joint_cmd_;

public:
    DownState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        kp_ = VecXf::Zero(cp_ptr_->dof_num_);
        kd_ = VecXf::Zero(cp_ptr_->dof_num_);
        if (robot_name_ == RobotName::DR2PRO) {
            kp_ << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_,
                cp_ptr_->neck_kp_;
            kd_ << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_,
                cp_ptr_->neck_kd_;
        }
    }

    void OnEnter() override {
        state_enter_time_ = static_cast<float>(ri_ptr_->GetInterfaceTimeStamp());
        joint_pos_init_ = ri_ptr_->GetJointPosition();
        joint_vel_init_ = ri_ptr_->GetJointVelocity();
        joint_pos_hold_ = joint_pos_init_;
        joint_pos_goal_ = joint_pos_init_;

        constexpr float kDegToRad = 0.01745329251994329577f;
        const std::array<float, kUpperBodyDof> down_pose{
            0.0f, 0.0f, 0.0f,
            0.0f, 7.0f * kDegToRad, 0.0f, 68.0f * kDegToRad, 0.0f, 0.0f, 0.0f,
            0.0f, -7.0f * kDegToRad, 0.0f, 68.0f * kDegToRad, 0.0f, 0.0f, 0.0f};
        const int count = std::min<int>(kUpperBodyDof, joint_pos_goal_.rows());
        for (int i = 0; i < count; ++i) {
            joint_pos_goal_(i) = down_pose[i];
        }
        motion_state_store_->SetState(RobotMotionState::DownMode);
    }

    void OnExit() override {}

    void Run() override {
        const float elapsed = std::max(0.0f, static_cast<float>(ri_ptr_->GetInterfaceTimeStamp()) - state_enter_time_);
        VecXf joint_pos_plan = joint_pos_hold_;
        VecXf joint_vel_plan = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf joint_tor_plan = VecXf::Zero(cp_ptr_->dof_num_);
        const int count = std::min<int>(kUpperBodyDof, joint_pos_plan.rows());

        for (int i = 0; i < count; ++i) {
            if (elapsed < kTransitionTime) {
                joint_pos_plan(i) = GetCubicSplinePos(joint_pos_init_(i), joint_vel_init_(i), joint_pos_goal_(i), 0.0f,
                                                       elapsed, kTransitionTime);
                joint_vel_plan(i) = GetCubicSplineVel(joint_pos_init_(i), joint_vel_init_(i), joint_pos_goal_(i), 0.0f,
                                                       elapsed, kTransitionTime);
            } else {
                joint_pos_plan(i) = joint_pos_goal_(i);
            }
        }

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(1) = joint_pos_plan;
        joint_cmd_.col(2) = kd_;
        joint_cmd_.col(3) = joint_vel_plan;
        joint_cmd_.col(4) = joint_tor_plan;
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    StateName GetNextStateName() override {
        const UserCommand* command = uc_ptr_->GetUserCommand();
        if (command->safe_control_mode != 0) {
            return StateName::kEmergency;
        }
        if (command->target_mode == uint8_t(RobotMotionState::Emergency)) {
            return StateName::kEmergency;
        }
        if (command->target_mode == uint8_t(RobotMotionState::ZeroPos)) {
            return StateName::kZeroPos;
        }
        return StateName::kDown;
    }
};

}  // namespace deep_robotics::dr02_pro
