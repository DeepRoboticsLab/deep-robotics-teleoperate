/**
 * @file user_command_interface.hpp
 * @brief User command input interface for the state machine.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <ctime>
#include <iostream>
#include <thread>

#include <termios.h>

#include "utils.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <unordered_map>

#include "types.h"
#include "motion_state_store.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#define KEYBOARD_REFRESH_MS 20

namespace deep_robotics::dr02_pro {

class UserCommandInterface {
 public:
  /**
   * @brief Construct a user command interface.
   * @param node ROS2 node used to create input subscribers.
   */
  explicit UserCommandInterface(rclcpp::Node::SharedPtr node)
      : node_(node) {
    if (!node_) {
      std::cerr << "UserCommandInterface node is null" << std::endl;
    }
    usr_cmd_ = new UserCommand();
    std::memset(usr_cmd_, 0, sizeof(UserCommand));

    gamepad_key_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/GAMEPAD_KEY", 10, std::bind(&UserCommandInterface::HandleKey, this, std::placeholders::_1));
  }

  virtual ~UserCommandInterface() { delete usr_cmd_; }

  /**
   * @brief Start keyboard and gamepad command handling.
   */
  virtual void Start() {
    std::memset(usr_cmd_, 0, sizeof(UserCommand));
    std::cout << "\n╔════════════════════════════════════════════════╗\n"
              << "║                GAMEPAD TELEOP                  ║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  Mode:      L1 (zeropos)  L2 (teleoperation)  R1 (emergency)  R2 (down)\n"
              << "╔════════════════════════════════════════════════╗\n"
              << "║                KEYBOARD TELEOP                 ║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  Mode:      Z (zeropos)  S (teleoperation)  D (down)  E (emergency) \n"
              << "\n";

    start_thread_flag_ = true;
    kb_thread_ = std::thread(std::bind(&UserCommandInterface::KeyboardLoop, this));
  }

  /**
   * @brief Stop keyboard command handling.
   */
  virtual void Stop() {
    start_thread_flag_ = false;
    if (kb_thread_.joinable()) {
      kb_thread_.join();
    }
  }

  /**
   * @brief Get the current user command.
   * @return Pointer to the current user command.
   */
  virtual UserCommand* GetUserCommand() { return usr_cmd_; }

  /**
   * @brief Set the shared motion-state store.
   * @param motion_state_store Shared motion-state store.
   */
  virtual void SetMotionStateStore(const std::shared_ptr<MotionStateStore>& motion_state_store) {
    motion_state_store_ = motion_state_store;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MotionStateStore> motion_state_store_;
  UserCommand* usr_cmd_;

 private:
  /**
   * @brief Get elapsed time since startup in milliseconds.
   * @return Elapsed time in milliseconds.
   */
  double GetCurrentTimeStamp() {
    static timespec startup_timestamp;
    timespec now_timestamp;
    if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
      clock_gettime(CLOCK_MONOTONIC, &startup_timestamp);
    }
    clock_gettime(CLOCK_MONOTONIC, &now_timestamp);
    return (now_timestamp.tv_sec - startup_timestamp.tv_sec) * 1e3 +
           (now_timestamp.tv_nsec - startup_timestamp.tv_nsec) / 1e6;
  }

  /**
   * @brief Get the current motion state.
   * @return Current robot motion state.
   */
  RobotMotionState GetCurrentState() const {
    return motion_state_store_ ? motion_state_store_->GetState() : RobotMotionState::Emergency;
  }

  /**
   * @brief Handle gamepad key messages from /GAMEPAD_KEY.
   * @param msg Received key message.
   */
  void HandleKey(const std_msgs::msg::String::SharedPtr msg) {
    static const std::unordered_map<std::string, KeyCode> key_map = {
        {"G20_KEY_L1", KeyCode::L1}, {"G20_KEY_L2", KeyCode::L2},
        {"G20_KEY_R1", KeyCode::R1}, {"G20_KEY_R2", KeyCode::R2}};

    KeyCode code = KeyCode::UNKNOWN;
    const auto it = key_map.find(msg->data);
    if (it != key_map.end()) {
      code = it->second;
    }

    const RobotMotionState current_state = GetCurrentState();
    switch (code) {
      case KeyCode::L1:
        if (current_state == RobotMotionState::Emergency || current_state == RobotMotionState::DownMode) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
          std::cout << "[GAMEPAD] L1 -> ZeroPos requested" << std::endl;
        } else {
          std::cout << "[GAMEPAD] L1 ignored: ZeroPos is available from Emergency or Down" << std::endl;
        }
        break;
      case KeyCode::L2:
        if (current_state == RobotMotionState::ZeroPos) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::TeleoperationMode);
          std::cout << "[GAMEPAD] L2 -> Teleoperation requested" << std::endl;
        } else {
          std::cout << "[GAMEPAD] L2 ignored: Teleoperation is available from ZeroPos" << std::endl;
        }
        break;
      case KeyCode::R1:
        usr_cmd_->target_mode = uint8_t(RobotMotionState::Emergency);
        std::cout << "[GAMEPAD] R1 -> Emergency pose hold requested" << std::endl;
        break;
      case KeyCode::R2:
        if (current_state == RobotMotionState::Emergency || current_state == RobotMotionState::TeleoperationMode) {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::DownMode);
          std::cout << "[GAMEPAD] R2 -> Down requested" << std::endl;
        } else {
          std::cout << "[GAMEPAD] R2 ignored: Down is available from Emergency or Teleoperation" << std::endl;
        }
        break;
      default:
        std::cout << "[GAMEPAD] ignored key: " << msg->data << std::endl;
        break;
    }
  }

  /**
   * @brief Poll keyboard input and update the current user command.
   */
  void KeyboardLoop() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    char input;
    while (start_thread_flag_) {
      usr_cmd_->time_stamp = GetCurrentTimeStamp();
      const RobotMotionState current_state = GetCurrentState();

      if (read(STDIN_FILENO, &input, 1) != -1) {
        const char key = static_cast<char>(std::tolower(static_cast<unsigned char>(input)));
        std::cout << "input: " << input << std::endl;
        if (key == 'e') {
          usr_cmd_->target_mode = uint8_t(RobotMotionState::Emergency);
          std::cout << "[MODE] Emergency pose hold" << std::endl;
        }
        switch (current_state) {
          case RobotMotionState::Emergency:
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
            } else if (key == 'd') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::DownMode);
            }
            break;
          case RobotMotionState::ZeroPos:
            if (key == 's') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::TeleoperationMode);
            }
            break;
          case RobotMotionState::TeleoperationMode:
            if (key == 'd') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::DownMode);
            }
            break;
          case RobotMotionState::DownMode:
            if (key == 'z') {
              usr_cmd_->target_mode = uint8_t(RobotMotionState::ZeroPos);
            }
            break;
          default:
            break;
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(KEYBOARD_REFRESH_MS));
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }

  bool start_thread_flag_ = false;
  std::thread kb_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gamepad_key_sub_;
};
}  // namespace deep_robotics::dr02_pro
