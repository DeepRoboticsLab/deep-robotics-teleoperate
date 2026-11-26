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

    struct SolveResult {
        Eigen::VectorXd q;
        Eigen::Matrix<double, 6, 1> left_error{
            Eigen::Matrix<double, 6, 1>::Zero()};
        Eigen::Matrix<double, 6, 1> right_error{
            Eigen::Matrix<double, 6, 1>::Zero()};
        double seed_task_cost{0.0};
        double task_cost{0.0};
        int iterations{0};
        int accepted_steps{0};
        bool converged{false};
    };

    Dr2ArmIK();

    Eigen::VectorXd ToArmQ(const std::array<float, kDof>& q) const;
    void WriteArmQ(const Eigen::VectorXd& arm_q, std::array<float, kDof>& q) const;
    /** Clamp arm positions; reject non-finite commands without modifying q. */
    bool ClampArmCommand(std::array<float, kDof>& q) const;
    void UpdateFK(const Eigen::VectorXd& q);
    Eigen::Matrix4d LeftPose() const;
    Eigen::Matrix4d RightPose() const;
    SolveResult Solve(const Eigen::Matrix4d& target_left,
                      const Eigen::Matrix4d& target_right,
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
