/**
 * @file idle_state.hpp
 * @brief Emergency pose-hold state for the state machine.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once
#include "state_base.h"

namespace deep_robotics::dr02_pro {
class EmergencyState : public StateBase {
private:
    VecXf joint_pos_hold_;
    VecXf kp_, kd_;
    MatXf joint_cmd_;

public:
    EmergencyState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
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
    /**
     * @brief Enter the emergency state.
     */
    void OnEnter() override {
        // Keep the entry pose as the position target. Replacing it with every
        // feedback sample would cancel the position error and leave only damping.
        joint_pos_hold_ = ri_ptr_->GetJointPosition();
        motion_state_store_->SetState(RobotMotionState::Emergency);
    };

    /**
     * @brief Exit the emergency state.
     */
    void OnExit() override {}

    /**
     * @brief Publish the emergency pose-hold joint command.
     */
    void Run() override {
        VecXf joint_pos_plan = joint_pos_hold_;
        VecXf joint_vel_plan = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf joint_tor_plan = VecXf::Zero(cp_ptr_->dof_num_);

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;

        joint_cmd_.col(1) = joint_pos_plan;
        joint_cmd_.col(3) = joint_vel_plan;
        joint_cmd_.col(4) = joint_tor_plan;
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    /**
     * @brief Get the next state requested from emergency.
     * @return Next state name.
     */
    StateName GetNextStateName() override {

        if (uc_ptr_->GetUserCommand()->safe_control_mode.load() != 0) {
            return StateName::kEmergency;
        }

        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::ZeroPos)) {
            return StateName::kZeroPos;
        }

        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::DownMode)) {
            return StateName::kDown;
        }

        return StateName::kEmergency;
    }
};

}  // namespace deep_robotics::dr02_pro
