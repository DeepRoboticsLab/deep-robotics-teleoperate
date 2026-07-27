/**
 * @file teleoperation_state.hpp
 * @brief Pico teleoperation state for upper-body control.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>

#include "pico_teleoperation.hpp"
#include "state_base.h"

namespace deep_robotics::dr02_pro {

class TeleoperationState : public StateBase {
private:
    static constexpr int kDofNum = PicoTeleoperationController::kDof;
    static constexpr int kUpperBodyDofNum = 17;

    PicoTeleoperationController teleop_;
    VecXf joint_pos_;
    VecXf joint_pos_hold_;
    MatXf joint_cmd_;
    VecXf kp_;
    VecXf kd_;
    bool seeded_{false};

    /**
     * @brief Read the latest robot joint state.
     */
    void GetRobotJointValue() {
        joint_pos_ = ri_ptr_->GetJointPosition();
    }

    /**
     * @brief Publish a no-motion hold command using the same layout as ZeroPosState.
     */
    void PublishHoldCommand() {
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
     * @brief Convert current robot joint positions to a fixed-size array.
     */
    std::array<float, kDofNum> GetCurrentJointArray() {
        std::array<float, kDofNum> current{};
        GetRobotJointValue();
        const int count = std::min<int>(kDofNum, joint_pos_.rows());
        for (int i = 0; i < count; ++i) {
            current[i] = joint_pos_(i);
        }
        return current;
    }

public:
    TeleoperationState(const RobotName& robot_name, const std::string& state_name,
                       std::shared_ptr<ControllerData> data_ptr)
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

    ~TeleoperationState() = default;

    /**
     * @brief Enter Pico teleoperation.
     */
    void OnEnter() override {
        if (cp_ptr_->dof_num_ != kDofNum) {
            std::cerr << "TeleoperationState expects " << kDofNum << " joints, got " << cp_ptr_->dof_num_
                      << std::endl;
        }
        const auto current = GetCurrentJointArray();
        joint_pos_hold_ = joint_pos_;
        PublishHoldCommand();
        teleop_.SeedFromCurrent(current);
        const int pico_start_result = teleop_.Start();
        if (pico_start_result != 0) {
            std::cerr << "Teleoperation state: PXREAInit failed with " << pico_start_result
                      << "; holding the entry pose." << std::endl;
        }
        seeded_ = true;
        motion_state_store_->SetState(RobotMotionState::TeleoperationMode);
        std::cout << "Teleoperation state entered. Hold Pico triggers to drive arms; headset drives waist."
                  << std::endl;
    }

    /**
     * @brief Exit Pico teleoperation.
     */
    void OnExit() override {
        teleop_.Stop();
        seeded_ = false;
    }

    /**
     * @brief Run one teleoperation update and publish joint command.
     */
    void Run() override {
        const auto control_start = std::chrono::steady_clock::now();
        const auto current = GetCurrentJointArray();
        if (!seeded_) {
            teleop_.SeedFromCurrent(current);
            seeded_ = true;
        }

        const auto command = teleop_.Update(current);
        if (!command.ready) {
            PublishHoldCommand();
            teleop_.RecordControlLoopDuration(
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - control_start).count());
            return;
        }

        // Keep legs and neck at the entry pose. Pico teleoperation only owns
        // waist and arm joints; using their latest feedback as a target would
        // cancel their position control and make them soft.
        VecXf joint_pos_plan = joint_pos_hold_;
        VecXf joint_vel_plan = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf joint_tor_plan = VecXf::Zero(cp_ptr_->dof_num_);

        const int upper_body_dof = std::min({cp_ptr_->dof_num_, kDofNum, kUpperBodyDofNum});
        for (int i = 0; i < upper_body_dof; ++i) {
            joint_pos_plan(i) = command.position[i];
            joint_vel_plan(i) = command.velocity[i];
        }

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;

        joint_cmd_.col(1) = joint_pos_plan;
        joint_cmd_.col(3) = joint_vel_plan;
        joint_cmd_.col(4) = joint_tor_plan;
        ri_ptr_->SetJointCommand(joint_cmd_);
        teleop_.RecordControlLoopDuration(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - control_start).count());
    }

    /**
     * @brief Get the next state requested from teleoperation.
     */
    StateName GetNextStateName() override {
        if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) {
            std::cout << "safe_control_mode:" << std::dec << uc_ptr_->GetUserCommand()->safe_control_mode
                      << std::endl;
            return StateName::kEmergency;
        }
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::Emergency)) {
            return StateName::kEmergency;
        }
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::DownMode)) {
            return StateName::kDown;
        }
        return StateName::kTeleoperation;
    }
};

}  // namespace deep_robotics::dr02_pro
