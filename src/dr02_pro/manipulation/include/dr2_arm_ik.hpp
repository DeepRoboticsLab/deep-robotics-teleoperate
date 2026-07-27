/**
 * @file dr2_arm_ik.hpp
 * @brief DR02 upper-body URDF based dual-arm IK helper.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <array>
#include <filesystem>
#include <string>

#include <Eigen/Dense>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

namespace deep_robotics::dr02_pro {

class Dr2ArmIK {
public:
    static constexpr int kDof = 31;

    Dr2ArmIK();

    Eigen::VectorXd ToArmQ(const std::array<float, kDof>& q) const;
    void WriteArmQ(const Eigen::VectorXd& arm_q, std::array<float, kDof>& q) const;
    void UpdateFK(const Eigen::VectorXd& q);
    Eigen::Matrix4d LeftPose() const;
    Eigen::Matrix4d RightPose() const;
    Eigen::VectorXd Solve(const Eigen::Matrix4d& target_left, const Eigen::Matrix4d& target_right,
                          const Eigen::VectorXd& seed);

private:
    static std::filesystem::path ResolveUrdfPath();

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex left_ee_id_;
    pinocchio::FrameIndex right_ee_id_;
    pinocchio::FrameIndex left_elbow_id_;
    pinocchio::FrameIndex right_elbow_id_;
};

}  // namespace deep_robotics::dr02_pro
