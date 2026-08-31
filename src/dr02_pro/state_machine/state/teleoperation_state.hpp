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
#include <optional>

#include "end_effector.hpp"
#include "pico_data_receiver.hpp"
#include "pico_teleoperation.hpp"
#include "recording_manager.hpp"
#include "state_base.h"

namespace deep_robotics::dr02_pro {

class TeleoperationState : public StateBase {
private:
    static constexpr int kDofNum = PicoTeleoperationController::kDof;
    static constexpr int kUpperBodyDofNum = 17;
    static constexpr int kArmDofPerSide = 7;
    static constexpr int kRecordedArmDof = 14;

    PicoDataReceiver pico_receiver_;
    PicoTeleoperationController teleop_;
    VecXf joint_pos_;
    VecXf joint_pos_hold_;
    MatXf joint_cmd_;
    VecXf kp_;
    VecXf kd_;
    bool seeded_{false};

    static double WallTimestampMs() {
        return static_cast<double>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
    }

    void HandleRecordingEvents(const PicoTeleoperationController::Command& command) {
        const auto& recorder = data_ptr_->recording_manager;
        if (!recorder || !recorder->IsEnabled()) {
            return;
        }
        if (command.left_primary_pressed) {
            if (recorder->GetStatus() == RecordingManager::Status::Recording) {
                recorder->RequestFinishEpisode();
            } else {
                if (data_ptr_->end_effector &&
                    !data_ptr_->end_effector->GetRecordingSample()) {
                    std::cout << "[Recording] start rejected: fresh end-effector feedback is required"
                              << std::endl;
                    return;
                }
                recorder->RequestStartEpisode();
            }
        }
        if (command.left_secondary_pressed) {
            recorder->RequestDeleteLastEpisode();
        }
    }

    void SubmitRecordingState(const VecXf& published_joint_position) {
        const auto& recorder = data_ptr_->recording_manager;
        if (!recorder || !recorder->IsRecording() ||
            joint_pos_.rows() < kUpperBodyDofNum ||
            published_joint_position.rows() < kUpperBodyDofNum) {
            return;
        }

        std::optional<EndEffectorSample> end_effector_sample;
        if (data_ptr_->end_effector) {
            end_effector_sample =
                data_ptr_->end_effector->GetRecordingSample();
            if (!end_effector_sample) {
                recorder->RequestAbortEpisode(
                    "end-effector feedback lost or stale");
                return;
            }
        }

        RecordingStateFrame frame;
        frame.timestamp_ms = WallTimestampMs();
        frame.qpos.resize(kRecordedArmDof);
        frame.action.resize(kRecordedArmDof);
        for (int i = 0; i < kArmDofPerSide; ++i) {
            frame.qpos[i] = static_cast<double>(joint_pos_(3 + i));
            frame.action[i] =
                static_cast<double>(published_joint_position(3 + i));
            frame.qpos[kArmDofPerSide + i] =
                static_cast<double>(joint_pos_(10 + i));
            frame.action[kArmDofPerSide + i] =
                static_cast<double>(published_joint_position(10 + i));
        }

        if (end_effector_sample) {
            frame.qpos.insert(frame.qpos.end(),
                              end_effector_sample->qpos.begin(),
                              end_effector_sample->qpos.end());
            frame.action.insert(frame.action.end(),
                                end_effector_sample->action.begin(),
                                end_effector_sample->action.end());
        }
        recorder->AddStateFrame(frame);
    }

    static EndEffectorInput MakeEndEffectorInput(const PicoState& pico) {
        EndEffectorInput input;
        input.sequence = pico.state_sequence;
        input.received_steady_ns = pico.received_steady_ns;
        input.left_valid = pico.left.valid;
        input.right_valid = pico.right.valid;
        input.left_grip = pico.left.grip;
        input.right_grip = pico.right.grip;
        return input;
    }

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
                       std::shared_ptr<ControllerData> data_ptr,
                       bool periodic_log_enabled = false)
        : StateBase(robot_name, state_name, data_ptr),
          teleop_(periodic_log_enabled) {
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
        teleop_.Reset();
        teleop_.SeedFromCurrent(current);
        const int pico_start_result = pico_receiver_.Start();
        if (pico_start_result != 0) {
            std::cerr << "Teleoperation state: PXREAInit failed with " << pico_start_result
                      << "; holding the entry pose." << std::endl;
        }
        const PicoState initial_pico = pico_receiver_.Snapshot();
        if (data_ptr_->end_effector) {
            data_ptr_->end_effector->OnTeleoperationEnter(
                initial_pico.state_sequence);
        }
        seeded_ = true;
        motion_state_store_->SetState(RobotMotionState::TeleoperationMode);
        std::cout << "Teleoperation state entered. Hold Pico triggers to drive arms; headset drives waist";
        if (data_ptr_->end_effector) {
            std::cout << "; grips drive " << data_ptr_->end_effector->Name();
        }
        std::cout << '.' << std::endl;
    }

    /**
     * @brief Exit Pico teleoperation.
     */
    void OnExit() override {
        if (data_ptr_->recording_manager) {
            data_ptr_->recording_manager->RequestAbortEpisode(
                "left Teleoperation before normal episode completion");
        }
        if (data_ptr_->end_effector) {
            data_ptr_->end_effector->OnTeleoperationExit();
        }
        pico_receiver_.Stop();
        teleop_.Reset();
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

        const PicoState pico = pico_receiver_.Snapshot();
        const auto command = teleop_.Update(current, pico);
        if (data_ptr_->end_effector) {
            data_ptr_->end_effector->Update(MakeEndEffectorInput(pico));
        }
        HandleRecordingEvents(command);
        if (!command.ready) {
            PublishHoldCommand();
            SubmitRecordingState(joint_pos_hold_);
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
        SubmitRecordingState(joint_pos_plan);
        teleop_.RecordControlLoopDuration(
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - control_start).count());
    }

    /**
     * @brief Get the next state requested from teleoperation.
     */
    StateName GetNextStateName() override {
        if (uc_ptr_->GetUserCommand()->safe_control_mode.load() != 0) {
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
