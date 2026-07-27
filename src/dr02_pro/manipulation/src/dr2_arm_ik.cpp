/**
 * @file dr2_arm_ik.cpp
 * @brief DR02 upper-body URDF based dual-arm IK helper.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#include "dr2_arm_ik.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/config.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace deep_robotics::dr02_pro {
namespace {

constexpr int kLeftArmStart = 3;
constexpr int kRightArmStart = 10;
constexpr int kIkMaxIterations = 50;
constexpr double kIkTolerance = 1e-4;
constexpr double kIkDamping = 1e-3;
constexpr double kPositionWeight = 50.0;
constexpr double kRotationWeight = 8.0;
constexpr double kElbowCylinderWeight = 5.0;
constexpr double kElbowCylinderRadius = 0.25;
constexpr double kJointRegularization = 0.01;
constexpr double kSeedRegularization = 0.1;
constexpr double kMaxIkStepNorm = 0.1;

double Clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

#ifndef DR02_PRO_REPOSITORY_ROOT
#define DR02_PRO_REPOSITORY_ROOT ""
#endif

}  // namespace

Dr2ArmIK::Dr2ArmIK() {
    const std::filesystem::path urdf_path = ResolveUrdfPath();
    if (!std::filesystem::exists(urdf_path)) {
        throw std::runtime_error("teleoperation URDF does not exist: " + urdf_path.string());
    }

    pinocchio::Model full_model;
    pinocchio::urdf::buildModel(urdf_path.string(), full_model);

    std::vector<pinocchio::JointIndex> locked_joint_ids;
    const std::vector<std::string> locked_joint_names = {
        "neck_y_joint",
        "neck_z_joint",
        "waist_z_joint",
        "waist_y_joint",
        "waist_x_joint",
        "left_hip_y_joint",
        "left_hip_x_joint",
        "left_hip_z_joint",
        "left_knee_joint",
        "left_ankle_y_joint",
        "left_ankle_x_joint",
        "right_hip_y_joint",
        "right_hip_x_joint",
        "right_hip_z_joint",
        "right_knee_joint",
        "right_ankle_y_joint",
        "right_ankle_x_joint",
    };

    for (const auto& name : locked_joint_names) {
        if (!full_model.existJointName(name)) {
            throw std::runtime_error("required joint is missing from the teleoperation URDF: " + name);
        }
        locked_joint_ids.push_back(full_model.getJointId(name));
    }

    model_ = pinocchio::buildReducedModel(full_model, locked_joint_ids, Eigen::VectorXd::Zero(full_model.nq));

    // Pinocchio 2.x requires both the parent joint and parent frame here,
    // whereas Pinocchio 3.x uses the four-argument overload.  In both cases
    // attach the end-effector frame to the wrist joint with a 0.15 m offset.
    const auto left_wrist_joint_id = model_.getJointId("left_wrist_x_joint");
    const auto right_wrist_joint_id = model_.getJointId("right_wrist_x_joint");
    const pinocchio::SE3 tool_offset(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.0, 0.0, 0.15));

#if PINOCCHIO_VERSION_AT_LEAST(3, 0, 0)
    model_.addFrame(
        pinocchio::Frame("L_ee", left_wrist_joint_id, tool_offset, pinocchio::OP_FRAME));
    model_.addFrame(
        pinocchio::Frame("R_ee", right_wrist_joint_id, tool_offset, pinocchio::OP_FRAME));
#else
    const auto left_wrist_frame_id = model_.getFrameId("left_wrist_x_joint");
    const auto right_wrist_frame_id = model_.getFrameId("right_wrist_x_joint");
    model_.addFrame(
        pinocchio::Frame("L_ee", left_wrist_joint_id, left_wrist_frame_id, tool_offset, pinocchio::OP_FRAME));
    model_.addFrame(
        pinocchio::Frame("R_ee", right_wrist_joint_id, right_wrist_frame_id, tool_offset, pinocchio::OP_FRAME));
#endif

    data_ = pinocchio::Data(model_);
    left_ee_id_ = model_.getFrameId("L_ee");
    right_ee_id_ = model_.getFrameId("R_ee");
    left_elbow_id_ = model_.getFrameId("left_elbow_link");
    right_elbow_id_ = model_.getFrameId("right_elbow_link");

    if (model_.nq != 14) {
        throw std::runtime_error("teleoperation IK expects 14 arm dof after locking waist, neck, and legs; got " +
                                 std::to_string(model_.nq));
    }
    std::cout << "[Dr2ArmIK] URDF: " << urdf_path.string() << std::endl;
    std::cout << "[Dr2ArmIK] full nq: " << full_model.nq << ", lock joints: " << locked_joint_ids.size()
              << ", reduced nq: " << model_.nq << std::endl;
}

Eigen::VectorXd Dr2ArmIK::ToArmQ(const std::array<float, kDof>& q) const {
    Eigen::VectorXd arm_q(14);
    for (int i = 0; i < 7; ++i) {
        arm_q(i) = q[kLeftArmStart + i];
        arm_q(i + 7) = q[kRightArmStart + i];
    }
    return arm_q;
}

void Dr2ArmIK::WriteArmQ(const Eigen::VectorXd& arm_q, std::array<float, kDof>& q) const {
    if (arm_q.size() < 14) {
        return;
    }
    for (int i = 0; i < 7; ++i) {
        q[kLeftArmStart + i] = static_cast<float>(arm_q(i));
        q[kRightArmStart + i] = static_cast<float>(arm_q(i + 7));
    }
}

void Dr2ArmIK::UpdateFK(const Eigen::VectorXd& q) {
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
}

Eigen::Matrix4d Dr2ArmIK::LeftPose() const {
    return data_.oMf[left_ee_id_].toHomogeneousMatrix();
}

Eigen::Matrix4d Dr2ArmIK::RightPose() const {
    return data_.oMf[right_ee_id_].toHomogeneousMatrix();
}

Eigen::VectorXd Dr2ArmIK::Solve(const Eigen::Matrix4d& target_left, const Eigen::Matrix4d& target_right,
                                const Eigen::VectorXd& seed) {
    Eigen::VectorXd q = seed;
    const pinocchio::SE3 target_l(target_left.block<3, 3>(0, 0), target_left.block<3, 1>(0, 3));
    const pinocchio::SE3 target_r(target_right.block<3, 3>(0, 0), target_right.block<3, 1>(0, 3));

    for (int iter = 0; iter < kIkMaxIterations; ++iter) {
        pinocchio::forwardKinematics(model_, data_, q);
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::updateFramePlacements(model_, data_);

        pinocchio::Data::Matrix6x j_l(6, model_.nv);
        pinocchio::Data::Matrix6x j_r(6, model_.nv);
        j_l.setZero();
        j_r.setZero();
        pinocchio::getFrameJacobian(model_, data_, left_ee_id_, pinocchio::LOCAL, j_l);
        pinocchio::getFrameJacobian(model_, data_, right_ee_id_, pinocchio::LOCAL, j_r);

        const Eigen::VectorXd err_l = pinocchio::log6(data_.oMf[left_ee_id_].actInv(target_l)).toVector();
        const Eigen::VectorXd err_r = pinocchio::log6(data_.oMf[right_ee_id_].actInv(target_r)).toVector();

        Eigen::MatrixXd h = Eigen::MatrixXd::Zero(model_.nv, model_.nv);
        Eigen::VectorXd g = Eigen::VectorXd::Zero(model_.nv);
        for (int i = 0; i < 6; ++i) {
            const double w = i < 3 ? kPositionWeight : kRotationWeight;
            h += w * j_l.row(i).transpose() * j_l.row(i);
            h += w * j_r.row(i).transpose() * j_r.row(i);
            g += w * j_l.row(i).transpose() * err_l(i);
            g += w * j_r.row(i).transpose() * err_r(i);
        }

        // Keep both elbows outside a cylinder centered on the torso. This is
        // the collision-avoidance term used by the validated Orin teleop IK.
        const auto add_elbow_cylinder_cost = [&](pinocchio::FrameIndex frame_id) {
            pinocchio::Data::Matrix6x j_elbow(6, model_.nv);
            j_elbow.setZero();
            pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, j_elbow);

            const Eigen::Vector3d elbow_pos = data_.oMf[frame_id].translation();
            const double radius_squared = elbow_pos.x() * elbow_pos.x() + elbow_pos.y() * elbow_pos.y();
            const double error = std::max(0.0, kElbowCylinderRadius * kElbowCylinderRadius - radius_squared);
            if (error <= 0.0) {
                return;
            }

            const Eigen::RowVectorXd jacobian =
                2.0 * elbow_pos.x() * j_elbow.row(0) + 2.0 * elbow_pos.y() * j_elbow.row(1);
            h += kElbowCylinderWeight * jacobian.transpose() * jacobian;
            g += kElbowCylinderWeight * jacobian.transpose() * error;
        };
        add_elbow_cylinder_cost(left_elbow_id_);
        add_elbow_cylinder_cost(right_elbow_id_);

        Eigen::VectorXd seed_weight = Eigen::VectorXd::Ones(model_.nv) * kSeedRegularization;
        seed_weight(3) *= 5.0;
        seed_weight(10) *= 5.0;

        Eigen::VectorXd joint_weight = Eigen::VectorXd::Ones(model_.nv) * kJointRegularization;

        h.diagonal().array() += kIkDamping;
        h.diagonal() += seed_weight;
        g -= joint_weight.asDiagonal() * q;
        g -= seed_weight.asDiagonal() * (q - seed);

        Eigen::VectorXd dq = h.ldlt().solve(g);
        if (!dq.allFinite()) {
            break;
        }
        if (dq.norm() > kMaxIkStepNorm) {
            dq = dq.normalized() * kMaxIkStepNorm;
        }

        q = pinocchio::integrate(model_, q, dq);
        for (int i = 0; i < model_.nq; ++i) {
            q(i) = Clamp(q(i), model_.lowerPositionLimit(i), model_.upperPositionLimit(i));
        }
        if (dq.norm() < kIkTolerance && err_l.norm() < 1e-2 && err_r.norm() < 1e-2) {
            break;
        }
    }
    return q;
}

std::filesystem::path Dr2ArmIK::ResolveUrdfPath() {
    const char* env_path = std::getenv("DR02_PRO_TELEOP_URDF");
    if (env_path != nullptr && env_path[0] != '\0') {
        return std::filesystem::path(env_path);
    }

    if (std::string(DR02_PRO_REPOSITORY_ROOT).empty()) {
        return std::filesystem::current_path() / "deep_robotics_model" / "DR02" / "urdf" / "pro" /
               "DR02-pro.urdf";
    }

    return std::filesystem::path(DR02_PRO_REPOSITORY_ROOT) / "deep_robotics_model" / "DR02" / "urdf" / "pro" /
           "DR02-pro.urdf";
}

}  // namespace deep_robotics::dr02_pro
