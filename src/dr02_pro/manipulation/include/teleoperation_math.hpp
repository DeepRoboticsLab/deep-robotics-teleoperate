/**
 * @file teleoperation_math.hpp
 * @brief Continuous input-shaping helpers for Pico teleoperation.
 */
#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Geometry>

namespace deep_robotics::dr02_pro::teleoperation_math {

inline double ApplyContinuousDeadband(double value, double deadband) {
    const double magnitude = std::abs(value);
    if (!std::isfinite(magnitude) || magnitude <= deadband) {
        return 0.0;
    }
    return std::copysign(magnitude - deadband, value);
}

inline Eigen::Vector3d ApplyContinuousRadialDeadband(
    const Eigen::Vector3d& value, double deadband, double scale,
    double maximum_norm) {
    const double input_norm = value.norm();
    if (!std::isfinite(input_norm) || input_norm <= deadband) {
        return Eigen::Vector3d::Zero();
    }

    const double output_norm =
        std::min((input_norm - deadband) * scale, maximum_norm);
    return value * (output_norm / input_norm);
}

inline Eigen::Matrix3d ApplyContinuousRotationDeadband(
    const Eigen::Matrix3d& rotation, double deadband,
    double maximum_angle) {
    const Eigen::AngleAxisd angle_axis(rotation);
    const double input_angle = angle_axis.angle();
    if (!std::isfinite(input_angle) || input_angle <= deadband) {
        return Eigen::Matrix3d::Identity();
    }

    const double output_angle =
        std::min(input_angle - deadband, maximum_angle);
    return Eigen::AngleAxisd(output_angle, angle_axis.axis())
        .toRotationMatrix();
}

}  // namespace deep_robotics::dr02_pro::teleoperation_math
