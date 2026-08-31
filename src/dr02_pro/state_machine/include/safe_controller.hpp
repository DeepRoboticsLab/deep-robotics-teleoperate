/**
 * @file safe_controller.hpp
 * @brief Runtime safety monitor for the state machine.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */

/**
 * I. Joint side
 *      1. Joint data abnormal (non-finite values)
 *      2. Joint data not updating
 *      3. Joint over-temperature
 */

#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "utils.hpp"
#include "time.hpp"
#include "robot_interface.hpp"
#include "user_command_interface.hpp"

namespace deep_robotics::dr02_pro {

class SafeController {
private:
    static constexpr double kJointFeedbackEmergencyTimeoutMs = 100.0;

    std::shared_ptr<RobotInterface> ri_ptr_;
    LoopTimer loop_timer_{};
    std::thread judge_thread_;
    int run_cnt_ = 0;

    double ri_check_time_{0.0};
    double last_ri_ts_{0.0};
    bool joint_check_flag_ = true;
    bool have_joint_feedback_ = false;

    UserCommand* usr_cmd_{nullptr};
    std::atomic<bool> start_thread_flag_{false};

    double current_time_{0.0};

    /**
     * @brief Check whether joint data is updating and finite.
     * @return true if joint data is normal; false otherwise.
     */
    bool IsJointDataNormal() {
        double ri_ts_ = ri_ptr_->GetInterfaceTimeStamp();
        VecXf joint_pos = ri_ptr_->GetJointPosition();
        VecXf joint_vel = ri_ptr_->GetJointVelocity();
        VecXf joint_tau = ri_ptr_->GetJointTorque();
        if (joint_check_flag_) {
            joint_check_flag_ = false;
            last_ri_ts_ = ri_ts_;
            ri_check_time_ = current_time_;
            have_joint_feedback_ = ri_ts_ != 0.0;
        }
        bool res = true;
        if (last_ri_ts_ != ri_ts_) {
            last_ri_ts_ = ri_ts_;
            ri_check_time_ = current_time_;
            have_joint_feedback_ = true;
        }
        if (have_joint_feedback_) {
            if (current_time_ - ri_check_time_ >
                kJointFeedbackEmergencyTimeoutMs) {
                res = false;
            }
        }

        for (int i = 0; i < joint_pos.size(); ++i) {
            if (!std::isfinite(joint_pos(i)) ||
                !std::isfinite(joint_vel(i)) ||
                !std::isfinite(joint_tau(i))) {
                res = false;
            }
        }
        return res;
    }

    /**
     * @brief Check whether motor temperatures are normal.
     * @return true if motor temperatures are normal; false otherwise.
     */
    bool IsMotorTemperatureNormal() {
        VecXf m_t = ri_ptr_->GetMotorTemperature();
        float max_temp = m_t.maxCoeff();

        // Motor over-temperature protection threshold: 110 degC
        if (max_temp > 110) {
            return false;
        }

        return true;
    }

public:
    SafeController() = default;
    ~SafeController() = default;

    /**
     * @brief Set the robot data source.
     * @param ri Robot interface.
     */
    void SetRobotDataSource(std::shared_ptr<RobotInterface> ri) {
        ri_ptr_ = ri;
    }
    /**
     * @brief Set the user command data source.
     * @param uc User command interface.
     */
    void SetUserCommandDataSource(std::shared_ptr<UserCommandInterface> uc) {
        usr_cmd_ = uc->GetUserCommand();
    }

    /**
     * @brief Start the safety monitor thread.
     */
    void Start() {
        start_thread_flag_ = true;
        judge_thread_ = std::thread(std::bind(&SafeController::Run, this));
    }

    /**
     * @brief Run the safety monitor loop.
     */
    void Run() {
        loop_timer_.Start(std::chrono::milliseconds(2));
        while (start_thread_flag_.load()) {
            if (loop_timer_.Wait() == 0) {
                continue;
            }
            run_cnt_++;
            current_time_ = GetTimestampMs();

            if (!IsJointDataNormal()) {
                usr_cmd_->safe_control_mode.store(3);
            }
            if (run_cnt_ % 1000 == 0 && !IsMotorTemperatureNormal()) {
                usr_cmd_->safe_control_mode.store(2);
            }
        }
    }

    /**
     * @brief Stop the safety monitor thread.
     */
    void Stop() {
        start_thread_flag_ = false;
        if (judge_thread_.joinable()) {
            judge_thread_.join();
        }
    }

};

}  // namespace deep_robotics::dr02_pro
