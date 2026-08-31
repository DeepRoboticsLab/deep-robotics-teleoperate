#pragma once

#include <memory>
#include <stdexcept>
#include <string>

#include "end_effector.hpp"
#include "zx_gripper/zhixing_end_effector.hpp"

namespace deep_robotics::dr02_pro {

inline bool IsSupportedEndEffector(const std::string& name) {
    return name.empty() || name == "zhixing";
}

inline std::unique_ptr<EndEffector> CreateEndEffector(
    const std::string& name) {
    if (name.empty()) {
        return nullptr;
    }
    if (name == "zhixing") {
        return std::make_unique<ZhixingEndEffector>();
    }
    throw std::invalid_argument("unsupported end effector: " + name);
}

}  // namespace deep_robotics::dr02_pro
