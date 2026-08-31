/**
 * @file utils.hpp
 * @brief Math and interpolation utility functions.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <cmath>
namespace deep_robotics::common {

/**
 * @brief Evaluate cubic spline position.
 * @param x0 Initial position.
 * @param v0 Initial velocity.
 * @param xf Final position.
 * @param vf Final velocity.
 * @param t Current time.
 * @param T Total interpolation time.
 * @return Interpolated position.
 */
inline float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T) {
    if (t >= T) return xf;
    float a, b, c, d;
    d = x0;
    c = v0;
    a = (vf * T - 2 * xf + v0 * T + 2 * x0) / std::pow(T, 3);
    b = (3 * xf - vf * T - 2 * v0 * T - 3 * x0) / std::pow(T, 2);
    return a * std::pow(t, 3) + b * std::pow(t, 2) + c * t + d;
}

/**
 * @brief Evaluate cubic spline velocity.
 * @param x0 Initial position.
 * @param v0 Initial velocity.
 * @param xf Final position.
 * @param vf Final velocity.
 * @param t Current time.
 * @param T Total interpolation time.
 * @return Interpolated velocity.
 */
inline float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T) {
    if (t >= T) return 0;
    float a, b, c;
    c = v0;
    a = (vf * T - 2 * xf + v0 * T + 2 * x0) / std::pow(T, 3);
    b = (3 * xf - vf * T - 2 * v0 * T - 3 * x0) / std::pow(T, 2);
    return 3.0f * a * std::pow(t, 2) + 2.0f * b * t + c;
}

}  // namespace deep_robotics::common
