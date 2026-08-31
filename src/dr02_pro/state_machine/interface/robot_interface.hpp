/**
 * @file robot_interface.hpp
 * @brief Robot data and joint-command interface for the state machine.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <iostream>
#include <string>

#include <unistd.h>

#include <functional>

#include "time.hpp"
#include "types.h"
#include "drdds/msg/joints.hpp"
#include "drdds/msg/joints_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

namespace deep_robotics::dr02_pro {
class RobotInterface {
protected:
    static constexpr uint16_t kMotorControlWord = 4;

    // Timing information
    double ri_ts_{0.0};  // Robot interface timestamp (seconds)

    // Joint data
    VecXf joint_pos_;           // Joint positions (rad)
    VecXf joint_vel_;           // Joint velocities (rad/s)
    VecXf joint_tau_;           // Joint torques (Nm)
    VecXf motor_temperature_;   // Motor temperatures (°C)

    // Status monitoring
    uint64_t joint_cmd_frame_id_{0};  // Joint command frame counter

    rclcpp::Publisher<drdds::msg::JointsCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<drdds::msg::Joints>::SharedPtr joint_data_sub_;

    int run_cnt_ = 0;  // Operation cycle counter

public:
    /**
     * @brief Construct a robot interface.
     * @param robot_name Robot name.
     * @param dof_num Total robot degree-of-freedom count.
     * @param node ROS2 node used to create publishers and subscribers.
     */
    RobotInterface(const std::string& robot_name, int dof_num, int leg_dof_num, int arm_dof_num, int waist_dof_num,
                   int neck_dof_num, rclcpp::Node::SharedPtr node)
        : dof_num_(dof_num),
          node_(node) {
        std::cout << robot_name << std::endl;
        std::cout << "Total DOF: " << dof_num_ << std::endl;
        std::cout << "  Leg DOF: " << leg_dof_num << std::endl;
        std::cout << "  Arm DOF: " << arm_dof_num << std::endl;
        std::cout << "Waist DOF: " << waist_dof_num << std::endl;
        std::cout << " Neck DOF: " << neck_dof_num << std::endl;

        if (!node_) {
            std::cerr << "RobotInterface node is null" << std::endl;
        }

        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        motor_temperature_ = VecXf::Zero(dof_num_);
    }

    virtual ~RobotInterface() = default;

    const int dof_num_;
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief Start joint-state and command topic interfaces.
     */
    virtual void Start() {
        joint_cmd_pub_ = node_->create_publisher<drdds::msg::JointsCmd>("/JOINTS_CMD", 10);
        const auto joint_data_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
        joint_data_sub_ = node_->create_subscription<drdds::msg::Joints>(
            "/JOINTS_DATA", joint_data_qos,
            std::bind(&RobotInterface::HandleJointData, this, std::placeholders::_1));
        WaitDataUpdate();
    }

    /**
     * @brief Get the robot interface timestamp.
     * @return Interface timestamp in seconds.
     */
    virtual double GetInterfaceTimeStamp() { return ri_ts_; }

    /**
     * @brief Get current joint positions.
     * @return Joint position vector.
     */
    virtual VecXf GetJointPosition() { return joint_pos_; }

    /**
     * @brief Get current joint velocities.
     * @return Joint velocity vector.
     */
    virtual VecXf GetJointVelocity() { return joint_vel_; }

    /**
     * @brief Get current joint torques.
     * @return Joint torque vector.
     */
    virtual VecXf GetJointTorque() { return joint_tau_; }

    /**
     * @brief Publish a joint command matrix.
     * @param input Joint command matrix with columns for kp, position, kd, velocity, and feedforward torque.
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        auto msg = drdds::msg::JointsCmd();
        const rclcpp::Time stamp = node_->now();
        const int64_t nanoseconds = stamp.nanoseconds();
        msg.header.frame_id = joint_cmd_frame_id_++;
        msg.header.stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
        msg.header.stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
        msg.data.resize(dof_num_);

        for (int i = 0; i < dof_num_; ++i) {
            msg.data[i].data_id = static_cast<uint16_t>(i);
            msg.data[i].control_word = kMotorControlWord;
            msg.data[i].position = input(i, 1);
            msg.data[i].velocity = input(i, 3);
            msg.data[i].torque = input(i, 4);
            msg.data[i].kp = input(i, 0);
            msg.data[i].kd = input(i, 2);
        }

        joint_cmd_pub_->publish(msg);
    }

    /**
     * @brief Get current motor temperatures.
     * @return Motor temperature vector.
     */
    virtual VecXf GetMotorTemperature() { return motor_temperature_; }

    /**
     * @brief Wait until the first joint-state frame is received.
     */
    virtual void WaitDataUpdate() {
        int cnt = 0;
        std::cout << "wait data update" << std::endl;
        while (rclcpp::ok()) {
            ++cnt;
            rclcpp::spin_some(node_);
            if (run_cnt_ > 0) {
                std::cout << "data updated at " << cnt << " cnt!" << std::endl;
                break;
            }
            usleep(1000);
            if (cnt == 10000) {
                std::cout << "joint data update is not finished\n";
            }
        }
    }

    /**
     * @brief Handle joint-state messages from /JOINTS_DATA.
     * @param msg Received joint-state message.
     */
    virtual void HandleJointData(const drdds::msg::Joints::SharedPtr msg) {
        ++run_cnt_;
        for (int i = 0; i < dof_num_; ++i) {
            joint_pos_(i) = msg->data[i].position;
            joint_vel_(i) = msg->data[i].velocity;
            joint_tau_(i) = msg->data[i].torque;
            motor_temperature_(i) = float(msg->data[i].motion_temp);
        }

        ri_ts_ = GetTimestampMs() / 1000.;
    }

};

}  // namespace deep_robotics::dr02_pro
