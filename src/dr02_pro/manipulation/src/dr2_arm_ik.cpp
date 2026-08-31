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
constexpr int kIkBacktrackingAttempts = 6;
constexpr double kIkTolerance = 1e-4;
constexpr double kIkInitialDamping = 1e-2;
constexpr double kIkMinimumDamping = 1e-6;
constexpr double kIkMaximumDamping = 1e4;
constexpr double kPositionWeight = 50.0;
constexpr double kRotationWeight = 8.0;
constexpr double kElbowCylinderWeight = 5.0;
constexpr double kElbowCylinderRadius = 0.25;
constexpr double kJointRegularization = 0.002;
constexpr double kSeedRegularization = 0.02;
constexpr double kMaxIkStepNorm = 0.05;

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

Dr2ArmIK::SolveResult Dr2ArmIK::Solve(
    const Eigen::Matrix4d& target_left,
    const Eigen::Matrix4d& target_right,
    const Eigen::VectorXd& seed) {
    if (seed.size() != model_.nq) {
        throw std::invalid_argument(
            "teleoperation IK seed size does not match reduced model");
    }

    SolveResult result;
    result.q = seed;
    Eigen::VectorXd q = seed;
    const pinocchio::SE3 target_l(target_left.block<3, 3>(0, 0), target_left.block<3, 1>(0, 3));
    const pinocchio::SE3 target_r(target_right.block<3, 3>(0, 0), target_right.block<3, 1>(0, 3));

    Eigen::VectorXd seed_weight =
        Eigen::VectorXd::Ones(model_.nv) * kSeedRegularization;
    seed_weight(3) *= 5.0;
    seed_weight(10) *= 5.0;
    Eigen::VectorXd joint_weight =
        Eigen::VectorXd::Ones(model_.nv) * kJointRegularization;

    const auto arm_task_cost =
        [](const Eigen::Matrix<double, 6, 1>& error) {
            return 0.5 * kPositionWeight *
                       error.head<3>().squaredNorm() +
                   0.5 * kRotationWeight *
                       error.tail<3>().squaredNorm();
        };
    const auto task_cost = [&](const Eigen::Matrix<double, 6, 1>& err_l,
                               const Eigen::Matrix<double, 6, 1>& err_r) {
        return arm_task_cost(err_l) + arm_task_cost(err_r);
    };
    const auto elbow_cost = [&](pinocchio::FrameIndex frame_id) {
        const Eigen::Vector3d elbow_pos =
            data_.oMf[frame_id].translation();
        const double radius_squared =
            elbow_pos.x() * elbow_pos.x() +
            elbow_pos.y() * elbow_pos.y();
        const double error = std::max(
            0.0, kElbowCylinderRadius * kElbowCylinderRadius -
                     radius_squared);
        return 0.5 * kElbowCylinderWeight * error * error;
    };
    const auto objective_from_current_fk =
        [&](const Eigen::VectorXd& candidate,
            const Eigen::Matrix<double, 6, 1>& err_l,
            const Eigen::Matrix<double, 6, 1>& err_r) {
            const Eigen::VectorXd seed_delta = candidate - seed;
            return task_cost(err_l, err_r) +
                   elbow_cost(left_elbow_id_) +
                   elbow_cost(right_elbow_id_) +
                   0.5 *
                       (joint_weight.array() *
                        candidate.array().square())
                           .sum() +
                   0.5 *
                       (seed_weight.array() *
                        seed_delta.array().square())
                           .sum();
        };
    const auto evaluate =
        [&](const Eigen::VectorXd& candidate,
            Eigen::Matrix<double, 6, 1>& err_l,
            Eigen::Matrix<double, 6, 1>& err_r) {
            pinocchio::forwardKinematics(model_, data_, candidate);
            pinocchio::updateFramePlacements(model_, data_);
            err_l = pinocchio::log6(
                        data_.oMf[left_ee_id_].actInv(target_l))
                        .toVector();
            err_r = pinocchio::log6(
                        data_.oMf[right_ee_id_].actInv(target_r))
                        .toVector();
            return objective_from_current_fk(candidate, err_l, err_r);
        };

    Eigen::Matrix<double, 6, 1> seed_err_l;
    Eigen::Matrix<double, 6, 1> seed_err_r;
    const double seed_objective =
        evaluate(seed, seed_err_l, seed_err_r);
    result.seed_task_cost = task_cost(seed_err_l, seed_err_r);

    Eigen::VectorXd best_q = seed;
    Eigen::Matrix<double, 6, 1> best_err_l = seed_err_l;
    Eigen::Matrix<double, 6, 1> best_err_r = seed_err_r;
    double best_objective = seed_objective;
    double best_task_cost = result.seed_task_cost;
    double damping = kIkInitialDamping;

    pinocchio::Data::Matrix6x j_l(6, model_.nv);
    pinocchio::Data::Matrix6x j_r(6, model_.nv);
    Eigen::MatrixXd h(model_.nv, model_.nv);
    Eigen::VectorXd g(model_.nv);

    for (int iter = 0; iter < kIkMaxIterations; ++iter) {
        result.iterations = iter + 1;
        Eigen::Matrix<double, 6, 1> err_l;
        Eigen::Matrix<double, 6, 1> err_r;
        const double current_objective = evaluate(q, err_l, err_r);
        const double current_task_cost = task_cost(err_l, err_r);
        if (!std::isfinite(current_objective) ||
            !std::isfinite(current_task_cost) ||
            !err_l.allFinite() || !err_r.allFinite()) {
            break;
        }
        if (current_objective < best_objective) {
            best_objective = current_objective;
            best_task_cost = current_task_cost;
            best_q = q;
            best_err_l = err_l;
            best_err_r = err_r;
        }

        pinocchio::computeJointJacobians(model_, data_, q);

        j_l.setZero();
        j_r.setZero();
        pinocchio::getFrameJacobian(model_, data_, left_ee_id_, pinocchio::LOCAL, j_l);
        pinocchio::getFrameJacobian(model_, data_, right_ee_id_, pinocchio::LOCAL, j_r);

        const pinocchio::SE3 error_l =
            data_.oMf[left_ee_id_].actInv(target_l);
        const pinocchio::SE3 error_r =
            data_.oMf[right_ee_id_].actInv(target_r);
        pinocchio::Data::Matrix6 jlog_l;
        pinocchio::Data::Matrix6 jlog_r;
        pinocchio::Jlog6(error_l.inverse(), jlog_l);
        pinocchio::Jlog6(error_r.inverse(), jlog_r);
        const pinocchio::Data::Matrix6x j_err_l = -jlog_l * j_l;
        const pinocchio::Data::Matrix6x j_err_r = -jlog_r * j_r;

        h.setZero();
        g.setZero();
        for (int i = 0; i < 6; ++i) {
            const double w = i < 3 ? kPositionWeight : kRotationWeight;
            h += w * j_err_l.row(i).transpose() * j_err_l.row(i);
            h += w * j_err_r.row(i).transpose() * j_err_r.row(i);
            g -= w * j_err_l.row(i).transpose() * err_l(i);
            g -= w * j_err_r.row(i).transpose() * err_r(i);
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

        h.diagonal().array() += damping;
        h.diagonal() += joint_weight;
        h.diagonal() += seed_weight;
        g -= joint_weight.asDiagonal() * q;
        g -= seed_weight.asDiagonal() * (q - seed);

        Eigen::LDLT<Eigen::MatrixXd> ldlt(h);
        if (ldlt.info() != Eigen::Success) {
            damping = std::min(kIkMaximumDamping, damping * 10.0);
            continue;
        }
        Eigen::VectorXd dq = ldlt.solve(g);
        if (ldlt.info() != Eigen::Success || !dq.allFinite()) {
            damping = std::min(kIkMaximumDamping, damping * 10.0);
            continue;
        }
        if (dq.norm() > kMaxIkStepNorm) {
            dq = dq.normalized() * kMaxIkStepNorm;
        }

        bool accepted = false;
        Eigen::VectorXd accepted_q = q;
        Eigen::Matrix<double, 6, 1> accepted_err_l = err_l;
        Eigen::Matrix<double, 6, 1> accepted_err_r = err_r;
        double accepted_objective = current_objective;
        double accepted_task_cost = current_task_cost;
        double scale = 1.0;
        for (int attempt = 0; attempt < kIkBacktrackingAttempts;
             ++attempt) {
            Eigen::VectorXd candidate =
                pinocchio::integrate(model_, q, dq * scale);
            for (int i = 0; i < model_.nq; ++i) {
                candidate(i) =
                    Clamp(candidate(i), model_.lowerPositionLimit(i),
                          model_.upperPositionLimit(i));
            }

            Eigen::Matrix<double, 6, 1> candidate_err_l;
            Eigen::Matrix<double, 6, 1> candidate_err_r;
            const double candidate_objective =
                evaluate(candidate, candidate_err_l, candidate_err_r);
            const double candidate_task_cost =
                task_cost(candidate_err_l, candidate_err_r);
            const double comparison_tolerance =
                1e-12 * std::max(1.0, current_objective);
            const double task_tolerance =
                1e-12 * std::max(1.0, current_task_cost);
            if (std::isfinite(candidate_objective) &&
                std::isfinite(candidate_task_cost) &&
                candidate_err_l.allFinite() &&
                candidate_err_r.allFinite() &&
                candidate_objective <=
                    current_objective + comparison_tolerance &&
                candidate_task_cost <=
                    current_task_cost + task_tolerance) {
                accepted = true;
                accepted_q = std::move(candidate);
                accepted_err_l = candidate_err_l;
                accepted_err_r = candidate_err_r;
                accepted_objective = candidate_objective;
                accepted_task_cost = candidate_task_cost;
                break;
            }
            scale *= 0.5;
        }

        if (!accepted) {
            damping = std::min(kIkMaximumDamping, damping * 10.0);
            if (damping >= kIkMaximumDamping) {
                break;
            }
            continue;
        }

        ++result.accepted_steps;
        if (accepted_objective < best_objective) {
            best_objective = accepted_objective;
            best_task_cost = accepted_task_cost;
            best_q = accepted_q;
            best_err_l = accepted_err_l;
            best_err_r = accepted_err_r;
        }

        const double applied_step_norm = (accepted_q - q).norm();
        q = std::move(accepted_q);
        damping =
            scale >= 0.999
                ? std::max(kIkMinimumDamping, damping * 0.5)
                : std::min(kIkMaximumDamping, damping * 2.0);
        if (applied_step_norm < kIkTolerance &&
            accepted_err_l.norm() < 1e-2 &&
            accepted_err_r.norm() < 1e-2) {
            result.converged = true;
            break;
        }
    }

    result.q = best_q;
    result.left_error = best_err_l;
    result.right_error = best_err_r;
    result.task_cost = best_task_cost;
    return result;
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
