#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace deep_robotics::dr02_pro {

struct EndEffectorInput {
    uint64_t sequence{0};
    uint64_t received_steady_ns{0};
    bool left_valid{false};
    bool right_valid{false};
    float left_grip{0.0F};
    float right_grip{0.0F};
};

struct EndEffectorSample {
    std::vector<double> qpos;
    std::vector<double> action;
};

class EndEffector {
public:
    virtual ~EndEffector() = default;

    virtual const std::string& Name() const = 0;
    virtual std::vector<std::string> StateNames() const = 0;
    virtual std::string RecordingLayout() const = 0;

    virtual bool Start(const rclcpp::Node::SharedPtr& node) = 0;
    virtual void Stop() = 0;
    virtual void OnTeleoperationEnter(uint64_t input_sequence) = 0;
    virtual void OnTeleoperationExit() = 0;
    virtual void Update(const EndEffectorInput& input) = 0;

    virtual bool FeedbackReady() const = 0;
    virtual std::optional<EndEffectorSample> GetRecordingSample() const = 0;
};

}  // namespace deep_robotics::dr02_pro
