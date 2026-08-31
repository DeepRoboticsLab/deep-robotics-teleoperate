#include "zhixing_end_effector.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

namespace deep_robotics::dr02_pro {

ZhixingEndEffector::~ZhixingEndEffector() {
    Stop();
}

const std::string& ZhixingEndEffector::Name() const {
    return name_;
}

std::vector<std::string> ZhixingEndEffector::StateNames() const {
    return {"left_zhixing_gripper", "right_zhixing_gripper"};
}

std::string ZhixingEndEffector::RecordingLayout() const {
    return "left_end_effector_1,right_end_effector_1";
}

bool ZhixingEndEffector::Start(const rclcpp::Node::SharedPtr& node) {
    if (command_publisher_ && feedback_subscription_) {
        return true;
    }
    if (!node) {
        std::cerr << "[ZhixingEndEffector] ROS node is null" << std::endl;
        return false;
    }
    node_ = node;
    command_publisher_ =
        node_->create_publisher<std_msgs::msg::Int32MultiArray>(
            "/gripper/position_cmd", 10);
    feedback_subscription_ =
        node_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/gripper/position", 10,
            std::bind(&ZhixingEndEffector::HandleFeedback, this,
                      std::placeholders::_1));
    std::cout << "[ZhixingEndEffector] ROS interfaces started: publish "
                 "/gripper/position_cmd, subscribe /gripper/position"
              << std::endl;
    return true;
}

void ZhixingEndEffector::Stop() {
    active_ = false;
    have_command_ = false;
    feedback_subscription_.reset();
    command_publisher_.reset();
    node_.reset();
    {
        std::lock_guard<std::mutex> lock(feedback_mutex_);
        have_left_feedback_ = false;
        have_right_feedback_ = false;
        last_left_feedback_ns_ = 0;
        last_right_feedback_ns_ = 0;
    }
}

void ZhixingEndEffector::OnTeleoperationEnter(uint64_t input_sequence) {
    active_ = true;
    have_command_ = false;
    entry_sequence_ = input_sequence;
    last_input_sequence_ = input_sequence;
    publish_counter_ = 0;
    last_warning_ns_ = 0;
    std::cout << "[ZhixingEndEffector] waiting for fresh Pico grip input"
              << std::endl;
}

void ZhixingEndEffector::OnTeleoperationExit() {
    active_ = false;
    have_command_ = false;
    publish_counter_ = 0;
}

void ZhixingEndEffector::Update(const EndEffectorInput& input) {
    if (!active_ || !command_publisher_) {
        return;
    }

    const uint64_t now_ns = SteadyNowNs();
    const bool new_sequence = input.sequence != last_input_sequence_;
    if (new_sequence) {
        last_input_sequence_ = input.sequence;
    }
    const bool valid_new_input =
        new_sequence && input.sequence != entry_sequence_ &&
        input.left_valid && input.right_valid &&
        IsFresh(input.received_steady_ns, now_ns, kInputFreshnessNs) &&
        std::isfinite(input.left_grip) && std::isfinite(input.right_grip);

    bool publish_immediately = false;
    if (valid_new_input) {
        const float left_grip = std::clamp(input.left_grip, 0.0F, 1.0F);
        const float right_grip = std::clamp(input.right_grip, 0.0F, 1.0F);
        left_position_ = static_cast<int32_t>(
            std::lround(left_grip * static_cast<float>(kMaxPosition)));
        right_position_ = static_cast<int32_t>(
            std::lround(right_grip * static_cast<float>(kMaxPosition)));
        left_action_ = 1.0 - static_cast<double>(left_grip);
        right_action_ = 1.0 - static_cast<double>(right_grip);
        publish_immediately = !have_command_;
        have_command_ = true;
    }

    if (publish_immediately) {
        PublishCommand();
        publish_counter_ = 0;
    } else if (have_command_ && ++publish_counter_ >= kPublishPeriod) {
        PublishCommand();
        publish_counter_ = 0;
    }

    ReportFeedbackWarning(now_ns);
}

bool ZhixingEndEffector::FeedbackReady() const {
    const uint64_t now_ns = SteadyNowNs();
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return have_left_feedback_ && have_right_feedback_ &&
           IsFresh(last_left_feedback_ns_, now_ns, kFeedbackTimeoutNs) &&
           IsFresh(last_right_feedback_ns_, now_ns, kFeedbackTimeoutNs);
}

std::optional<EndEffectorSample>
ZhixingEndEffector::GetRecordingSample() const {
    const uint64_t now_ns = SteadyNowNs();
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (!have_command_ || !have_left_feedback_ || !have_right_feedback_ ||
        !IsFresh(last_left_feedback_ns_, now_ns, kFeedbackTimeoutNs) ||
        !IsFresh(last_right_feedback_ns_, now_ns, kFeedbackTimeoutNs)) {
        return std::nullopt;
    }
    EndEffectorSample sample;
    sample.qpos = {
        1.0 - std::clamp(static_cast<double>(left_feedback_) / 1000.0,
                         0.0, 1.0),
        1.0 - std::clamp(static_cast<double>(right_feedback_) / 1000.0,
                         0.0, 1.0)};
    sample.action = {left_action_, right_action_};
    return sample;
}

uint64_t ZhixingEndEffector::SteadyNowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

bool ZhixingEndEffector::IsFresh(uint64_t timestamp_ns, uint64_t now_ns,
                                 uint64_t timeout_ns) {
    return timestamp_ns != 0 && now_ns >= timestamp_ns &&
           now_ns - timestamp_ns <= timeout_ns;
}

void ZhixingEndEffector::PublishCommand() {
    if (!command_publisher_ || !have_command_) {
        return;
    }
    std_msgs::msg::Int32MultiArray message;
    message.data = {kSideBoth,
                    std::clamp(left_position_, kMinPosition, kMaxPosition),
                    std::clamp(right_position_, kMinPosition, kMaxPosition)};
    command_publisher_->publish(message);
}

void ZhixingEndEffector::HandleFeedback(
    const std_msgs::msg::Int32MultiArray::SharedPtr message) {
    if (!message || message->data.size() != 2) {
        return;
    }
    const int32_t side = message->data[0];
    if (side != 0 && side != 1) {
        return;
    }
    const int32_t position = message->data[1];
    // RK and Orin clocks are not synchronized. Feedback freshness is based
    // only on the Orin-local steady-clock time at which this message arrives.
    const uint64_t now_ns = SteadyNowNs();
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    if (side == 0) {
        left_feedback_ = position;
        last_left_feedback_ns_ = now_ns;
        have_left_feedback_ = true;
    } else {
        right_feedback_ = position;
        last_right_feedback_ns_ = now_ns;
        have_right_feedback_ = true;
    }
}

void ZhixingEndEffector::ReportFeedbackWarning(uint64_t now_ns) {
    if (FeedbackReady()) {
        return;
    }
    if (last_warning_ns_ != 0 &&
        now_ns - last_warning_ns_ < kWarningPeriodNs) {
        return;
    }
    last_warning_ns_ = now_ns;
    std::cerr << "[ZhixingEndEffector] feedback unavailable or stale; "
                 "arm teleoperation continues"
              << std::endl;
}

}  // namespace deep_robotics::dr02_pro
