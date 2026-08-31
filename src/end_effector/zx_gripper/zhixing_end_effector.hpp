#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "end_effector.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

namespace deep_robotics::dr02_pro {

class ZhixingEndEffector final : public EndEffector {
public:
    static constexpr uint64_t kInputFreshnessNs = 200000000ULL;
    static constexpr uint64_t kFeedbackTimeoutNs = 200000000ULL;

    ZhixingEndEffector() = default;
    ~ZhixingEndEffector() override;

    const std::string& Name() const override;
    std::vector<std::string> StateNames() const override;
    std::string RecordingLayout() const override;

    bool Start(const rclcpp::Node::SharedPtr& node) override;
    void Stop() override;
    void OnTeleoperationEnter(uint64_t input_sequence) override;
    void OnTeleoperationExit() override;
    void Update(const EndEffectorInput& input) override;

    bool FeedbackReady() const override;
    std::optional<EndEffectorSample> GetRecordingSample() const override;

private:
    static uint64_t SteadyNowNs();
    static bool IsFresh(uint64_t timestamp_ns, uint64_t now_ns,
                        uint64_t timeout_ns);

    void PublishCommand();
    void HandleFeedback(
        const std_msgs::msg::Int32MultiArray::SharedPtr message);
    void ReportFeedbackWarning(uint64_t now_ns);

    static constexpr int32_t kSideBoth = 2;
    static constexpr int32_t kMinPosition = 0;
    static constexpr int32_t kMaxPosition = 1000;
    static constexpr int kPublishPeriod = 20;
    static constexpr uint64_t kWarningPeriodNs = 1000000000ULL;

    const std::string name_{"zhixing"};
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
        command_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
        feedback_subscription_;

    bool active_{false};
    bool have_command_{false};
    uint64_t entry_sequence_{0};
    uint64_t last_input_sequence_{0};
    uint64_t last_warning_ns_{0};
    int publish_counter_{0};
    double left_action_{0.0};
    double right_action_{0.0};
    int32_t left_position_{0};
    int32_t right_position_{0};

    mutable std::mutex feedback_mutex_;
    bool have_left_feedback_{false};
    bool have_right_feedback_{false};
    uint64_t last_left_feedback_ns_{0};
    uint64_t last_right_feedback_ns_{0};
    int32_t left_feedback_{0};
    int32_t right_feedback_{0};
};

}  // namespace deep_robotics::dr02_pro
