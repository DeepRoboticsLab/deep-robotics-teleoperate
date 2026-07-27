/**
 * @file humanoid_state_machine.hpp
 * @brief Humanoid state-machine controller.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "time.hpp"
#include "dr2_pro_interface.hpp"

#include "state_base.h"
#include "zeropos_state.hpp"
#include "idle_state.hpp"
#include "teleoperation_state.hpp"
#include "down_state.hpp"

#include "safe_controller.hpp"
#include "user_command_interface.hpp"

namespace deep_robotics::dr02_pro {
class HumanoidStateMachine {
private:
    std::shared_ptr<StateBase> emergency_controller_;
    std::shared_ptr<StateBase> zeropos_controller_;
    std::shared_ptr<StateBase> teleoperation_controller_;
    std::shared_ptr<StateBase> down_controller_;

public:
    int run_cnt_ = 0;
    LoopTimer loop_timer_{};
    std::shared_ptr<MotionStateStore> motion_state_store_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<SafeController> sc_ptr_;
    rclcpp::Node::SharedPtr node_;

    std::shared_ptr<StateBase> current_controller_;
    StateName current_state_name_, next_state_name_;

    std::thread run_thread_;

    // The loop remains at 2 ms to keep responsiveness, but command generation
    // is tied to fresh /JOINTS_DATA feedback just like the CR1 state machine.
    bool have_robot_feedback_timestamp_{false};
    double last_robot_feedback_timestamp_{0.0};

    const RobotName robot_name_;

    /**
     * @brief Construct a humanoid state machine.
     * @param robot_name Robot name.
     */
    HumanoidStateMachine(RobotName robot_name)
        : robot_name_(robot_name),
          motion_state_store_(std::make_shared<MotionStateStore>()) {}

    ~HumanoidStateMachine() {}

    /**
     * @brief Initialize interfaces, controllers, and the first state.
     */
    void Start() {
        node_ = std::make_shared<rclcpp::Node>("sdk_deploy");
        uc_ptr_ = std::make_shared<UserCommandInterface>(node_);
        uc_ptr_->SetMotionStateStore(motion_state_store_);

        ri_ptr_ = std::make_shared<DR2ProInterface>(node_);
        cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);

        std::cout << "HumanoidInterface finish" << std::endl;

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        data_ptr->motion_state_store = motion_state_store_;

        std::cout << "Control mode: teleoperation" << std::endl;

        sc_ptr_ = std::make_shared<SafeController>("");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);

        emergency_controller_ = std::make_shared<EmergencyState>(robot_name_, "emergency_state", data_ptr);
        zeropos_controller_ = std::make_shared<ZeroPosState>(robot_name_, "zeropos_state", data_ptr);
        teleoperation_controller_ = std::make_shared<TeleoperationState>(robot_name_, "teleoperation", data_ptr);
        down_controller_ = std::make_shared<DownState>(robot_name_, "down_state", data_ptr);

        current_controller_ = emergency_controller_;
        current_state_name_ = kEmergency;
        next_state_name_ = kEmergency;

        std::this_thread::sleep_for(std::chrono::seconds(3));  // for safety

        ri_ptr_->Start();
        uc_ptr_->Start();
#ifndef USE_SIMULATION
        sc_ptr_->Start();
#endif
        current_controller_->OnEnter();
        
        std::cout << "start finish" << std::endl;
    }

    /**
     * @brief Run the state-machine control loop.
     */
    void Run() {
        loop_timer_.Start(std::chrono::milliseconds(2));
        run_thread_ = std::thread(&HumanoidStateMachine::RunThread, this);
        
        while (rclcpp::ok()) {
            if (loop_timer_.Wait() == 0) {
                continue;
            }

            ri_ptr_->RefreshRobotData();

            const double robot_feedback_timestamp = ri_ptr_->GetInterfaceTimeStamp();
            if (have_robot_feedback_timestamp_ && robot_feedback_timestamp == last_robot_feedback_timestamp_) {
                continue;
            }
            last_robot_feedback_timestamp_ = robot_feedback_timestamp;
            have_robot_feedback_timestamp_ = true;

            current_controller_->Run();

            next_state_name_ = current_controller_->GetNextStateName();

            if (next_state_name_ != current_state_name_) {
                current_controller_->OnExit();
                std::cout << current_controller_->state_name_ << " ------------> ";
                current_controller_ = GetStateControllerPtr(next_state_name_);
                std::cout << current_controller_->state_name_ << std::endl;
                current_controller_->OnEnter();
                current_state_name_ = next_state_name_;
            }
            ++run_cnt_;
        }
    }

    /**
     * @brief Spin the ROS2 node in a background thread.
     */
    void RunThread() {
        if (rclcpp::ok()) {
            rclcpp::spin(node_);
        }
    }

    /**
     * @brief Get the controller object for a state name.
     * @param state_name Requested state name.
     * @return State controller pointer.
     */
    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) {
        switch (state_name) {
            case StateName::kInvalid: {
                return nullptr;
            }
            case StateName::kEmergency: {
                return emergency_controller_;
            }
            case StateName::kZeroPos: {
                return zeropos_controller_;
            }
            case StateName::kTeleoperation: {
                return teleoperation_controller_;
            }
            case StateName::kDown: {
                return down_controller_;
            }
            default: {
                std::cerr << "error state name" << std::endl;
                return emergency_controller_;
            }
        }
        return nullptr;
    }

    /**
     * @brief Stop the state-machine components.
     */
    void Stop() {
        if (run_thread_.joinable()) {
            run_thread_.join();
        }
        if (current_controller_) {
            current_controller_->OnExit();
        }
        sc_ptr_->Stop();
        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }
};
}  // namespace deep_robotics::dr02_pro
