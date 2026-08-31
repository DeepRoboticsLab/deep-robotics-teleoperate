/**
 * @file pico_teleoperation.cpp
 * @brief Pico teleoperation controller implementation.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#include "pico_teleoperation.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "dr2_arm_ik.hpp"
#include "pico_data_receiver.hpp"
#include "teleoperation_math.hpp"

namespace deep_robotics::dr02_pro {
namespace {

constexpr int kDof = PicoTeleoperationController::kDof;
constexpr int kArmDof = 14;
constexpr int kArmStart = 3;
constexpr int kLegStart = 17;
constexpr float kTriggerThreshold = 0.5f;
constexpr float kTransDeadband = 0.015f;
constexpr float kTransScale = 1.2f;
constexpr float kMaxHandTranslation = 0.5f;
constexpr double kRotationDeadband = 0.035;
constexpr double kMaxHandRotation = 1.45;
constexpr float kMaxStepPerTick = 0.006f;
constexpr double kHeadPoseEpsilon = 1e-6;
constexpr float kHeadPitchToWaistZ = 0.8f;
constexpr float kHeadYawToWaistX = 0.9f;
constexpr float kHeadRollToWaistY = 0.6f;
constexpr double kHeadAngleDeadband = 0.006;
constexpr double kHeadFilterAlpha = 0.88;

// Matches the CR1 teleoperation output path: a short weighted average first,
// followed by a critically damped second-order joint-space smoother.
constexpr std::array<double, 4> kMovingAverageWeights{0.1, 0.2, 0.3, 0.4};
constexpr double kSmootherOmega = 30.0;
constexpr double kMinFeedbackDt = 0.001;
constexpr double kMaxFeedbackDt = 0.005;
constexpr uint64_t kPicoSoftStaleNs = 200ULL * 1000ULL * 1000ULL;
constexpr uint64_t kPicoHardStaleNs = 1000ULL * 1000ULL * 1000ULL;
constexpr uint64_t kPicoRecoveryPacketCount = 3;
constexpr double kMaxFinalIkPositionErrorM = 0.05;
constexpr double kMaxFinalIkRotationErrorRad =
    15.0 * M_PI / 180.0;
constexpr double kMaxTargetAdvancePerSolveRad = 0.5;

double Clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

float Slew(float current, float target, float max_step) {
    const float delta = target - current;
    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

uint64_t SteadyNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

Eigen::Matrix3d QuatToRot(const std::array<double, 4>& q_xyzw) {
    Eigen::Quaterniond q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
    if (q.norm() < 1e-9) {
        return Eigen::Matrix3d::Identity();
    }
    return q.normalized().toRotationMatrix();
}

std::array<double, 3> QuatToRollPitchYaw(const std::array<double, 4>& q_xyzw) {
    Eigen::Quaterniond q(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
    if (q.norm() < kHeadPoseEpsilon) {
        return {0.0, 0.0, 0.0};
    }
    q.normalize();

    const double sinr_cosp = 2.0 * (q.w() * q.x() + q.y() * q.z());
    const double cosr_cosp = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    const double roll = std::atan2(sinr_cosp, cosr_cosp);

    const double sinp = 2.0 * (q.w() * q.y() - q.z() * q.x());
    const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);

    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    const double yaw = std::atan2(siny_cosp, cosy_cosp);

    return {roll, pitch, yaw};
}

double WrapAngle(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

}  // namespace

class PicoTeleoperationController::Impl {
public:
    explicit Impl(bool periodic_log_enabled)
        : periodic_log_enabled_(periodic_log_enabled) {
        lower_.fill(-3.2f);
        upper_.fill(3.2f);
        SetArmLimits(3, true);
        SetArmLimits(10, false);
        SetRange(0, -0.6f, 0.6f);
        SetRange(1, -0.4f, 0.4f);
        SetRange(2, -0.4f, 0.4f);
    }

    void Reset() {
        diagnostics_started_ = false;
        buttons_baselined_ = false;
        left_primary_last_ = false;
        left_secondary_last_ = false;
        have_state_ = false;
        left_latched_ = false;
        right_latched_ = false;
        head_latched_ = false;
        pico_stale_ = false;
        pico_hard_stale_ = false;
        trigger_release_required_ = false;
        pico_stale_entered_ns_ = 0;
        recovery_fresh_packet_count_ = 0;
        last_recovery_pico_sequence_ = 0;
        have_update_time_ = false;
        have_ik_pico_sequence_ = false;
        buttons_baselined_ = false;
        left_primary_last_ = false;
        left_secondary_last_ = false;
        ResetSmoother(Eigen::VectorXd::Zero(kArmDof));
        left_anchor_pos_.fill(0.0);
        right_anchor_pos_.fill(0.0);
        left_anchor_quat_ = {0.0, 0.0, 0.0, 1.0};
        right_anchor_quat_ = {0.0, 0.0, 0.0, 1.0};
        left_anchor_ee_.setIdentity();
        right_anchor_ee_.setIdentity();
        left_target_pose_.setIdentity();
        right_target_pose_.setIdentity();
    }

    void SeedFromCurrent(const std::array<float, kDof>& current) {
        if (!EnsureIk()) {
            have_state_ = false;
            return;
        }

        current_ = current;
        target_ = current;
        command_ = current;
        for (int i = 0; i < 3; ++i) {
            waist_target_[i] = current_[i];
        }
        const Eigen::VectorXd arm_q = ik_->ToArmQ(command_);
        raw_arm_target_ = arm_q;
        ik_->UpdateFK(arm_q);
        left_target_pose_ = ik_->LeftPose();
        right_target_pose_ = ik_->RightPose();
        ResetSmoother(arm_q);
        left_latched_ = false;
        right_latched_ = false;
        head_latched_ = false;
        filtered_head_delta_ = {0.0, 0.0, 0.0};
        have_update_time_ = false;
        have_ik_pico_sequence_ = false;
        have_state_ = true;
    }

    Command Update(const std::array<float, kDof>& current,
                   const PicoState& pico) {
        const auto update_start = std::chrono::steady_clock::now();
        Command out;

        const uint64_t now_ns = SteadyNowNs();
        const bool invalid_receive_time =
            pico.received_steady_ns == 0 || now_ns < pico.received_steady_ns;
        const uint64_t pico_age_ns = invalid_receive_time
                                         ? 0
                                         : now_ns - pico.received_steady_ns;
        const bool soft_stale_timeout =
            invalid_receive_time || pico_age_ns > kPicoSoftStaleNs;
        const bool hard_stale_timeout =
            (!invalid_receive_time && pico_age_ns > kPicoHardStaleNs) ||
            (invalid_receive_time && pico_stale_entered_ns_ != 0 &&
             now_ns - pico_stale_entered_ns_ > kPicoHardStaleNs);
        UpdateLeftButtonEvents(pico, soft_stale_timeout || pico_stale_, out);

        if (!EnsureIk()) {
            return out;
        }
        if (!have_state_) {
            SeedFromCurrent(current);
            if (!have_state_) {
                return out;
            }
        }

        const double feedback_dt = GetFeedbackDt(update_start);
        ++feedback_update_count_;
        current_ = current;

        if (soft_stale_timeout) {
            EnterStaleHold(hard_stale_timeout, pico.state_sequence, now_ns);
            UpdateTriggerStatus(pico, true);
            BuildOutput(out);
            ReportDiagnostics(pico, now_ns);
            return out;
        }

        if (pico_stale_ && !HasStablePicoRecovery(pico.state_sequence)) {
            UpdateTriggerStatus(pico, true);
            BuildOutput(out);
            ReportDiagnostics(pico, now_ns);
            return out;
        }

        UpdateTriggerStatus(pico, false);

        const bool any_trigger_held = (pico.left.valid && pico.left.trigger >= kTriggerThreshold) ||
                                      (pico.right.valid && pico.right.trigger >= kTriggerThreshold);
        if (trigger_release_required_) {
            if (!any_trigger_held) {
                trigger_release_required_ = false;
                std::cout << "[PicoTeleoperation] Pico stream recovered; release acknowledged" << std::endl;
            }
            BuildOutput(out);
            ReportDiagnostics(pico, now_ns);
            return out;
        }
        const bool left_just_latched =
            CaptureIfNeeded(pico.left, true, pico.left.trigger >= kTriggerThreshold, left_anchor_pos_, left_anchor_quat_,
                            left_anchor_ee_, left_latched_);
        const bool right_just_latched =
            CaptureIfNeeded(pico.right, false, pico.right.trigger >= kTriggerThreshold, right_anchor_pos_,
                            right_anchor_quat_, right_anchor_ee_, right_latched_);
        if (!left_just_latched) {
            ApplyController(pico.left, true, left_anchor_pos_, left_anchor_quat_, left_anchor_ee_);
        }
        if (!right_just_latched) {
            ApplyController(pico.right, false, right_anchor_pos_, right_anchor_quat_, right_anchor_ee_);
        }

        const bool fresh_pico_for_ik =
            !have_ik_pico_sequence_ ||
            pico.state_sequence != last_ik_pico_sequence_;

        // A trigger edge only establishes the hand-to-robot anchor. Solve at
        // most once per fresh Pico packet; the joint-space smoother continues
        // on every robot feedback frame using the cached raw IK solution.
        if (left_just_latched || right_just_latched) {
            last_ik_pico_sequence_ = pico.state_sequence;
            have_ik_pico_sequence_ = true;
        } else if (any_trigger_held && fresh_pico_for_ik) {
            const auto ik_start = std::chrono::steady_clock::now();
            const Eigen::VectorXd seed_q = ik_->ToArmQ(command_);
            const Dr2ArmIK::SolveResult solved =
                ik_->Solve(left_target_pose_, right_target_pose_, seed_q);
            last_ik_pico_sequence_ = pico.state_sequence;
            have_ik_pico_sequence_ = true;
            const double ik_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - ik_start)
                                     .count();
            ik_total_ms_ += ik_ms;
            ik_max_ms_ = std::max(ik_max_ms_, ik_ms);
            ++ik_solve_count_;

            const bool candidate_size_ok =
                solved.q.size() == kArmDof &&
                seed_q.size() == kArmDof;
            const double left_position_error_m =
                solved.left_error.head<3>().norm();
            const double right_position_error_m =
                solved.right_error.head<3>().norm();
            const double left_rotation_error_rad =
                solved.left_error.tail<3>().norm();
            const double right_rotation_error_rad =
                solved.right_error.tail<3>().norm();
            const double max_joint_delta_rad =
                candidate_size_ok
                    ? (solved.q - seed_q).cwiseAbs().maxCoeff()
                    : std::numeric_limits<double>::infinity();
            const double task_cost_tolerance =
                1e-9 * std::max(1.0, solved.seed_task_cost);
            const bool reject_candidate =
                !candidate_size_ok ||
                !solved.q.allFinite() ||
                !solved.left_error.allFinite() ||
                !solved.right_error.allFinite() ||
                !std::isfinite(solved.seed_task_cost) ||
                !std::isfinite(solved.task_cost) ||
                solved.task_cost >
                    solved.seed_task_cost + task_cost_tolerance ||
                left_position_error_m >
                    kMaxFinalIkPositionErrorM ||
                right_position_error_m >
                    kMaxFinalIkPositionErrorM ||
                left_rotation_error_rad >
                    kMaxFinalIkRotationErrorRad ||
                right_rotation_error_rad >
                    kMaxFinalIkRotationErrorRad;
            if (reject_candidate) {
                ++ik_rejected_count_;
                ++ik_rejected_total_;
                HoldArmAtLastCommand();
                if (ik_rejected_total_ == 1 ||
                    ik_rejected_total_ % 100 == 0) {
                    std::cerr
                        << "[PicoTeleoperation] reject IK candidate #"
                        << ik_rejected_total_
                        << ": pos_err(L/R)="
                        << left_position_error_m << '/'
                        << right_position_error_m
                        << " m, rot_err(L/R)="
                        << left_rotation_error_rad * 180.0 / M_PI
                        << '/'
                        << right_rotation_error_rad * 180.0 / M_PI
                        << " deg, max_joint_delta="
                        << max_joint_delta_rad
                        << " rad, task(seed/final)="
                        << solved.seed_task_cost << '/'
                        << solved.task_cost
                        << ", iterations=" << solved.iterations
                        << ", accepted_steps="
                        << solved.accepted_steps
                        << ", converged=" << solved.converged
                        << std::endl;
                }
            } else {
                raw_arm_target_ = solved.q;
                bool target_limited = false;
                for (int i = 0; i < kArmDof; ++i) {
                    const double delta =
                        solved.q(i) - seed_q(i);
                    const double limited_delta =
                        Clamp(delta, -kMaxTargetAdvancePerSolveRad,
                              kMaxTargetAdvancePerSolveRad);
                    raw_arm_target_(i) =
                        seed_q(i) + limited_delta;
                    target_limited =
                        target_limited ||
                        limited_delta != delta;
                }
                if (target_limited) {
                    ++ik_limited_count_;
                    ++ik_limited_total_;
                    if (ik_limited_total_ == 1 ||
                        ik_limited_total_ % 100 == 0) {
                        std::cout
                            << "[PicoTeleoperation] limit IK target #"
                            << ik_limited_total_
                            << ": max_joint_delta="
                            << max_joint_delta_rad
                            << " rad, per-joint advance limited to "
                            << kMaxTargetAdvancePerSolveRad
                            << " rad"
                            << std::endl;
                    }
                }
            }
        }
        const Eigen::VectorXd filtered_q =
            FilterArmTarget(raw_arm_target_, feedback_dt);
        ik_->WriteArmQ(filtered_q, target_);

        ApplyHeadPoseToWaist(pico.head);
        for (int i = 0; i < 3; ++i) {
            target_[i] = waist_target_[i];
        }
        for (int i = 0; i < kLegStart; ++i) {
            target_[i] = static_cast<float>(Clamp(target_[i], lower_[i], upper_[i]));
        }
        for (int i = kLegStart; i < kDof; ++i) {
            target_[i] = current_[i];
        }

        BuildOutput(out);
        ReportDiagnostics(pico, now_ns);
        return out;
    }

    void RecordControlLoopDuration(double milliseconds) {
        control_total_ms_ += milliseconds;
        control_max_ms_ = std::max(control_max_ms_, milliseconds);
        ++control_loop_count_;
    }

    void UpdateLeftButtonEvents(const PicoState& pico, bool stale, Command& out) {
        if (stale || !pico.left.valid) {
            // Re-baseline after a stale interval. A button held while entering
            // teleoperation or while the stream recovers must be released
            // before it can generate an event.
            buttons_baselined_ = false;
            return;
        }
        if (!buttons_baselined_) {
            left_primary_last_ = pico.left.primary;
            left_secondary_last_ = pico.left.secondary;
            buttons_baselined_ = true;
            return;
        }
        out.left_primary_pressed = pico.left.primary && !left_primary_last_;
        out.left_secondary_pressed = pico.left.secondary && !left_secondary_last_;
        left_primary_last_ = pico.left.primary;
        left_secondary_last_ = pico.left.secondary;
    }

private:
    bool EnsureIk() {
        if (ik_) {
            return true;
        }
        try {
            ik_ = std::make_unique<Dr2ArmIK>();
        } catch (const std::exception& e) {
            if (!ik_error_reported_) {
                std::cerr << "[PicoTeleoperation] failed to initialize IK: " << e.what() << std::endl;
                ik_error_reported_ = true;
            }
            return false;
        }
        return true;
    }

    void SetRange(int idx, float lo, float hi) {
        lower_[idx] = lo;
        upper_[idx] = hi;
    }

    void SetArmLimits(int start, bool left) {
        SetRange(start + 0, -1.2f, 1.4f);
        SetRange(start + 1, left ? -0.7f : -1.8f, left ? 1.8f : 0.7f);
        SetRange(start + 2, -1.6f, 1.6f);
        SetRange(start + 3, -0.2f, 1.8f);
        SetRange(start + 4, -2.6f, 2.6f);    // wrist-z
        SetRange(start + 5, -1.45f, 1.45f);  // wrist-y
        SetRange(start + 6, -1.45f, 1.45f);  // wrist-x
    }

    double GetFeedbackDt(const std::chrono::steady_clock::time_point& now) {
        double dt = kMaxFeedbackDt;
        if (have_update_time_) {
            dt = std::chrono::duration<double>(now - last_update_time_).count();
        }
        last_update_time_ = now;
        have_update_time_ = true;
        return Clamp(dt, kMinFeedbackDt, kMaxFeedbackDt);
    }

    void ResetSmoother(const Eigen::VectorXd& arm_q) {
        arm_history_size_ = 0;
        for (auto& sample : arm_history_) {
            sample.fill(0.0);
        }
        smoothed_arm_q_.fill(0.0);
        smoothed_arm_dq_.fill(0.0);
        if (arm_q.size() == kArmDof) {
            for (int i = 0; i < kArmDof; ++i) {
                smoothed_arm_q_[i] = arm_q(i);
            }
            smoother_initialized_ = true;
        } else {
            smoother_initialized_ = false;
        }
    }

    void HoldArmAtLastCommand() {
        const Eigen::VectorXd held_q = ik_->ToArmQ(command_);
        raw_arm_target_ = held_q;
        ResetSmoother(held_q);
        for (int i = 3; i < kLegStart; ++i) {
            target_[i] = command_[i];
        }
    }

    Eigen::VectorXd FilterArmTarget(const Eigen::VectorXd& raw_q, double dt) {
        if (raw_q.size() != kArmDof) {
            return raw_q;
        }
        for (size_t i = 0; i + 1 < arm_history_.size(); ++i) {
            arm_history_[i] = arm_history_[i + 1];
        }
        for (int i = 0; i < kArmDof; ++i) {
            arm_history_.back()[i] = raw_q(i);
        }
        arm_history_size_ = std::min(arm_history_size_ + 1, arm_history_.size());

        std::array<double, kArmDof> average{};
        const size_t first_sample =
            arm_history_.size() - arm_history_size_;
        double weight_sum = 0.0;
        for (size_t sample = first_sample;
             sample < arm_history_.size(); ++sample) {
            weight_sum += kMovingAverageWeights[sample];
            for (int joint = 0; joint < kArmDof; ++joint) {
                average[joint] +=
                    kMovingAverageWeights[sample] *
                    arm_history_[sample][joint];
            }
        }
        for (double& value : average) {
            value /= weight_sum;
        }

        if (!smoother_initialized_) {
            smoothed_arm_q_ = average;
            smoothed_arm_dq_.fill(0.0);
            smoother_initialized_ = true;
        } else {
            for (int joint = 0; joint < kArmDof; ++joint) {
                const double acceleration = kSmootherOmega * kSmootherOmega * (average[joint] - smoothed_arm_q_[joint]) -
                                            2.0 * kSmootherOmega * smoothed_arm_dq_[joint];
                smoothed_arm_dq_[joint] += acceleration * dt;
                smoothed_arm_q_[joint] += smoothed_arm_dq_[joint] * dt;
            }
        }

        Eigen::VectorXd filtered(kArmDof);
        for (int i = 0; i < kArmDof; ++i) {
            filtered(i) = smoothed_arm_q_[i];
        }
        return filtered;
    }

    bool CaptureIfNeeded(const PicoControllerState& controller, bool left, bool active,
                         std::array<double, 3>& anchor_pos, std::array<double, 4>& anchor_quat,
                         Eigen::Matrix4d& anchor_ee, bool& latched) {
        if (!active || !controller.valid) {
            latched = false;
            return false;
        }
        if (latched) {
            return false;
        }
        anchor_pos = controller.pos;
        anchor_quat = controller.quat;
        const Eigen::VectorXd arm_q = ik_->ToArmQ(command_);
        ik_->UpdateFK(arm_q);
        anchor_ee = left ? ik_->LeftPose() : ik_->RightPose();
        latched = true;
        std::cout << "[PicoTeleoperation] " << (left ? "left" : "right")
                  << " trigger latched; waiting for hand motion" << std::endl;
        return true;
    }

    void ApplyController(const PicoControllerState& controller, bool left, const std::array<double, 3>& anchor_pos,
                         const std::array<double, 4>& anchor_quat, const Eigen::Matrix4d& anchor_ee) {
        if (!controller.valid || controller.trigger < kTriggerThreshold) {
            return;
        }
        const double dx = controller.pos[0] - anchor_pos[0];
        const double dy = controller.pos[1] - anchor_pos[1];
        const double dz = controller.pos[2] - anchor_pos[2];
        const Eigen::Matrix3d r_align =
            (Eigen::Matrix3d() << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();

        const Eigen::Vector3d raw_delta_pos(dx, dy, dz);
        const Eigen::Vector3d delta_pos =
            r_align *
            teleoperation_math::ApplyContinuousRadialDeadband(
                raw_delta_pos, kTransDeadband, kTransScale,
                kMaxHandTranslation);

        const Eigen::Matrix3d anchor_r = QuatToRot(anchor_quat);
        const Eigen::Matrix3d current_r = QuatToRot(controller.quat);
        const Eigen::Matrix3d delta_r = anchor_r.transpose() * current_r;
        Eigen::Matrix3d robot_delta_r =
            r_align * delta_r * r_align.transpose();

        // The DR02 Pro model uses the same vertical-axis wrist convention as
        // the aligned Pico frame. Do not apply the legacy z-axis inversion
        // that was required by the previous arm URDF.
        robot_delta_r =
            teleoperation_math::ApplyContinuousRotationDeadband(
                robot_delta_r, kRotationDeadband, kMaxHandRotation);

        Eigen::Matrix4d target_pose = anchor_ee;
        target_pose.block<3, 1>(0, 3) = anchor_ee.block<3, 1>(0, 3) + delta_pos;
        target_pose.block<3, 3>(0, 0) = robot_delta_r * anchor_ee.block<3, 3>(0, 0);
        if (left) {
            left_target_pose_ = target_pose;
        } else {
            right_target_pose_ = target_pose;
        }
    }

    void ApplyHeadPoseToWaist(const PicoHeadState& head) {
        if (!head.valid) {
            return;
        }
        const std::array<double, 3> rpy = QuatToRollPitchYaw(head.quat);
        if (!head_latched_) {
            head_anchor_rpy_ = rpy;
            waist_anchor_ = waist_target_;
            filtered_head_delta_ = {0.0, 0.0, 0.0};
            head_latched_ = true;
            std::cout << "[PicoTeleoperation] latched headset pose for waist control" << std::endl;
            return;
        }

        const std::array<double, 3> raw_delta{
            teleoperation_math::ApplyContinuousDeadband(
                WrapAngle(rpy[0] - head_anchor_rpy_[0]),
                kHeadAngleDeadband),
            teleoperation_math::ApplyContinuousDeadband(
                WrapAngle(rpy[1] - head_anchor_rpy_[1]),
                kHeadAngleDeadband),
            teleoperation_math::ApplyContinuousDeadband(
                WrapAngle(rpy[2] - head_anchor_rpy_[2]),
                kHeadAngleDeadband)};
        for (int i = 0; i < 3; ++i) {
            filtered_head_delta_[i] =
                kHeadFilterAlpha * filtered_head_delta_[i] + (1.0 - kHeadFilterAlpha) * raw_delta[i];
        }
        waist_target_[0] = static_cast<float>(
            Clamp(waist_anchor_[0] + kHeadPitchToWaistZ * filtered_head_delta_[1], lower_[0], upper_[0]));
        waist_target_[1] = static_cast<float>(
            Clamp(waist_anchor_[1] - kHeadYawToWaistX * filtered_head_delta_[2], lower_[1], upper_[1]));
        waist_target_[2] = static_cast<float>(
            Clamp(waist_anchor_[2] - kHeadRollToWaistY * filtered_head_delta_[0], lower_[2], upper_[2]));
    }

    void BuildOutput(Command& out) {
        for (int i = 0; i < kDof; ++i) {
            command_[i] = (i >= kLegStart) ? current_[i] : Slew(command_[i], target_[i], kMaxStepPerTick);
            out.position[i] = command_[i];
            out.velocity[i] = 0.0f;
        }
        if (smoother_initialized_) {
            for (int i = 0; i < kArmDof; ++i) {
                out.velocity[kArmStart + i] =
                    static_cast<float>(smoothed_arm_dq_[i]);
            }
        }
        out.ready = true;
    }

    void EnterStaleHold(bool hard_stale, uint64_t state_sequence,
                        uint64_t now_ns) {
        recovery_fresh_packet_count_ = 0;
        last_recovery_pico_sequence_ = state_sequence;
        if (!pico_stale_) {
            std::cerr << "[PicoTeleoperation] Pico data stale (>200 ms); freezing the last safe target"
                      << std::endl;
            // command_ is the last command actually handed to the robot.
            // Do not continue slewing toward an older, unissued IK target.
            target_ = command_;
            waist_target_ = {command_[0], command_[1], command_[2]};
            const Eigen::VectorXd arm_q = ik_->ToArmQ(command_);
            raw_arm_target_ = arm_q;
            ResetSmoother(arm_q);
            ik_->UpdateFK(arm_q);
            left_target_pose_ = ik_->LeftPose();
            right_target_pose_ = ik_->RightPose();
            left_latched_ = false;
            right_latched_ = false;
            head_latched_ = false;
            have_ik_pico_sequence_ = false;
            pico_stale_ = true;
            pico_stale_entered_ns_ = now_ns;
        }
        if (hard_stale && !pico_hard_stale_) {
            pico_hard_stale_ = true;
            trigger_release_required_ = true;
            std::cerr << "[PicoTeleoperation] Pico data hard stale (>1000 ms); trigger release required"
                      << std::endl;
        }
    }

    bool HasStablePicoRecovery(uint64_t state_sequence) {
        if (state_sequence != last_recovery_pico_sequence_) {
            const uint64_t new_packets = state_sequence > last_recovery_pico_sequence_
                                             ? state_sequence - last_recovery_pico_sequence_
                                             : 1;
            recovery_fresh_packet_count_ = std::min(
                kPicoRecoveryPacketCount,
                recovery_fresh_packet_count_ + new_packets);
            last_recovery_pico_sequence_ = state_sequence;
        }
        if (recovery_fresh_packet_count_ < kPicoRecoveryPacketCount) {
            return false;
        }

        pico_stale_ = false;
        pico_hard_stale_ = false;
        pico_stale_entered_ns_ = 0;
        recovery_fresh_packet_count_ = 0;
        std::cout << "[PicoTeleoperation] Pico stream stable after "
                  << kPicoRecoveryPacketCount << " fresh packets" << std::endl;
        return true;
    }

    void UpdateTriggerStatus(const PicoState& pico, bool stale) {
        const bool left_active = !stale && pico.left.valid && pico.left.trigger >= kTriggerThreshold;
        const bool right_active = !stale && pico.right.valid && pico.right.trigger >= kTriggerThreshold;
        if (left_active != left_trigger_active_ || right_active != right_trigger_active_) {
            left_trigger_active_ = left_active;
            right_trigger_active_ = right_active;
            std::cout << "[PicoTeleoperation] trigger state: left=" << pico.left.trigger
                      << " right=" << pico.right.trigger << " stale=" << stale << std::endl;
        }
    }

    void ReportDiagnostics(const PicoState& pico, uint64_t now_ns) {
        const auto now = std::chrono::steady_clock::now();
        if (!diagnostics_started_) {
            diagnostics_start_ = now;
            diagnostics_started_ = true;
            last_valid_packet_count_ = pico.valid_packet_count;
            last_parse_error_count_ = pico.parse_error_count;
            return;
        }
        const double window_seconds = std::chrono::duration<double>(now - diagnostics_start_).count();
        if (window_seconds < 1.0) {
            return;
        }
        const double pico_age_ms = pico.received_steady_ns == 0 || now_ns < pico.received_steady_ns
                                       ? -1.0
                                       : static_cast<double>(now_ns - pico.received_steady_ns) / 1e6;
        const uint64_t valid_delta = pico.valid_packet_count - last_valid_packet_count_;
        const uint64_t parse_delta = pico.parse_error_count - last_parse_error_count_;
        const double ik_average_ms = ik_solve_count_ == 0 ? 0.0 : ik_total_ms_ / ik_solve_count_;
        const double control_average_ms = control_loop_count_ == 0 ? 0.0 : control_total_ms_ / control_loop_count_;
        if (periodic_log_enabled_) {
            std::cout << std::fixed << std::setprecision(2)
                      << "[PicoTeleoperation] diag pico_age_ms=" << pico_age_ms << " packets=" << valid_delta
                      << "(total=" << pico.valid_packet_count << ") parse_fail=" << parse_delta
                      << "(total=" << pico.parse_error_count << ") feedback_hz=" << feedback_update_count_ / window_seconds
                      << " ik_hz=" << ik_solve_count_ / window_seconds
                      << " ik_reject=" << ik_rejected_count_
                      << "(total=" << ik_rejected_total_ << ')'
                      << " ik_limit=" << ik_limited_count_
                      << "(total=" << ik_limited_total_ << ')'
                      << " ik_ms(avg/max)=" << ik_average_ms << '/' << ik_max_ms_
                      << " control_ms(avg/max)=" << control_average_ms << '/' << control_max_ms_
                      << " trigger(L/R)=" << pico.left.trigger << '/' << pico.right.trigger
                      << " grip(L/R)=" << pico.left.grip << '/' << pico.right.grip
                      << " stale=" << pico_stale_ << std::defaultfloat << std::endl;
        }

        diagnostics_start_ = now;
        last_valid_packet_count_ = pico.valid_packet_count;
        last_parse_error_count_ = pico.parse_error_count;
        feedback_update_count_ = 0;
        ik_solve_count_ = 0;
        ik_rejected_count_ = 0;
        ik_limited_count_ = 0;
        ik_total_ms_ = 0.0;
        ik_max_ms_ = 0.0;
        control_loop_count_ = 0;
        control_total_ms_ = 0.0;
        control_max_ms_ = 0.0;
    }

    const bool periodic_log_enabled_;
    bool have_state_{false};
    bool ik_error_reported_{false};
    std::unique_ptr<Dr2ArmIK> ik_;
    Eigen::VectorXd raw_arm_target_{
        Eigen::VectorXd::Zero(kArmDof)};
    std::array<float, kDof> current_{};
    std::array<float, kDof> target_{};
    std::array<float, kDof> command_{};
    std::array<float, kDof> lower_{};
    std::array<float, kDof> upper_{};
    std::array<float, 3> waist_target_{0.0f, 0.0f, 0.0f};

    std::array<std::array<double, kArmDof>, 4> arm_history_{};
    size_t arm_history_size_{0};
    std::array<double, kArmDof> smoothed_arm_q_{};
    std::array<double, kArmDof> smoothed_arm_dq_{};
    bool smoother_initialized_{false};
    std::chrono::steady_clock::time_point last_update_time_{};
    bool have_update_time_{false};
    uint64_t last_ik_pico_sequence_{0};
    bool have_ik_pico_sequence_{false};

    bool left_latched_{false};
    bool right_latched_{false};
    bool head_latched_{false};
    bool pico_stale_{false};
    bool pico_hard_stale_{false};
    bool trigger_release_required_{false};
    uint64_t pico_stale_entered_ns_{0};
    uint64_t recovery_fresh_packet_count_{0};
    uint64_t last_recovery_pico_sequence_{0};
    bool left_trigger_active_{false};
    bool right_trigger_active_{false};
    bool buttons_baselined_{false};
    bool left_primary_last_{false};
    bool left_secondary_last_{false};
    std::array<double, 3> left_anchor_pos_{};
    std::array<double, 3> right_anchor_pos_{};
    std::array<double, 4> left_anchor_quat_{};
    std::array<double, 4> right_anchor_quat_{};
    std::array<double, 3> head_anchor_rpy_{};
    std::array<double, 3> filtered_head_delta_{0.0, 0.0, 0.0};
    std::array<float, 3> waist_anchor_{0.0f, 0.0f, 0.0f};
    Eigen::Matrix4d left_anchor_ee_{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d right_anchor_ee_{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d left_target_pose_{Eigen::Matrix4d::Identity()};
    Eigen::Matrix4d right_target_pose_{Eigen::Matrix4d::Identity()};

    bool diagnostics_started_{false};
    std::chrono::steady_clock::time_point diagnostics_start_{};
    uint64_t last_valid_packet_count_{0};
    uint64_t last_parse_error_count_{0};
    uint64_t feedback_update_count_{0};
    uint64_t ik_solve_count_{0};
    uint64_t ik_rejected_count_{0};
    uint64_t ik_rejected_total_{0};
    uint64_t ik_limited_count_{0};
    uint64_t ik_limited_total_{0};
    uint64_t control_loop_count_{0};
    double ik_total_ms_{0.0};
    double ik_max_ms_{0.0};
    double control_total_ms_{0.0};
    double control_max_ms_{0.0};
};

PicoTeleoperationController::PicoTeleoperationController(
    bool periodic_log_enabled)
    : impl_(std::make_unique<Impl>(periodic_log_enabled)) {}
PicoTeleoperationController::~PicoTeleoperationController() = default;
void PicoTeleoperationController::Reset() { impl_->Reset(); }
void PicoTeleoperationController::SeedFromCurrent(const std::array<float, kDof>& current) { impl_->SeedFromCurrent(current); }
PicoTeleoperationController::Command PicoTeleoperationController::Update(
    const std::array<float, kDof>& current, const PicoState& pico) {
    return impl_->Update(current, pico);
}
void PicoTeleoperationController::RecordControlLoopDuration(double milliseconds) {
    impl_->RecordControlLoopDuration(milliseconds);
}
}  // namespace deep_robotics::dr02_pro
