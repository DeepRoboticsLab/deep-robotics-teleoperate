#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "zx_gripper/zhixing_end_effector.hpp"

namespace {

using deep_robotics::dr02_pro::EndEffectorInput;
using deep_robotics::dr02_pro::ZhixingEndEffector;
using Int32MultiArray = std_msgs::msg::Int32MultiArray;

uint64_t SteadyNowNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void SpinFor(rclcpp::executors::SingleThreadedExecutor& executor,
             std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto device_node = std::make_shared<rclcpp::Node>("zhixing_device_test");
        auto peer_node = std::make_shared<rclcpp::Node>("zhixing_peer_test");
        ZhixingEndEffector device;
        Require(device.Start(device_node), "device failed to start");

        std::vector<Int32MultiArray> commands;
        auto command_subscription = peer_node->create_subscription<Int32MultiArray>(
            "/gripper/position_cmd", 10,
            [&commands](const Int32MultiArray::SharedPtr message) {
                commands.push_back(*message);
            });
        auto feedback_publisher = peer_node->create_publisher<Int32MultiArray>(
            "/gripper/position", 10);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(device_node);
        executor.add_node(peer_node);
        SpinFor(executor, std::chrono::milliseconds(100));

        device.OnTeleoperationEnter(10);
        SpinFor(executor, std::chrono::milliseconds(30));
        Require(commands.empty(), "teleoperation entry published a command");

        EndEffectorInput input;
        input.sequence = 11;
        input.received_steady_ns = SteadyNowNs();
        input.left_valid = true;
        input.right_valid = true;
        input.left_grip = 0.0F;
        input.right_grip = 1.0F;
        device.Update(input);
        SpinFor(executor, std::chrono::milliseconds(100));

        Require(commands.size() == 1, "fresh grip did not publish one command");
        Require(commands[0].data == std::vector<int32_t>({2, 0, 1000}),
                "command is not [2, left, right]");

        Int32MultiArray feedback;
        feedback.data = {0, 0};
        feedback_publisher->publish(feedback);
        SpinFor(executor, std::chrono::milliseconds(50));
        Require(!device.FeedbackReady(),
                "one-sided feedback was treated as complete");

        feedback.data = {1, 1000};
        feedback_publisher->publish(feedback);
        SpinFor(executor, std::chrono::milliseconds(50));
        Require(device.FeedbackReady(), "two-sided feedback is not ready");

        const auto sample = device.GetRecordingSample();
        Require(sample.has_value(), "recording sample is unavailable");
        Require(sample->qpos.size() == 2 && sample->action.size() == 2,
                "recording sample width is incorrect");
        Require(std::abs(sample->qpos[0] - 1.0) < 1e-9 &&
                    std::abs(sample->qpos[1]) < 1e-9 &&
                    std::abs(sample->action[0] - 1.0) < 1e-9 &&
                    std::abs(sample->action[1]) < 1e-9,
                "recording value mapping is incorrect");

        device.Stop();
        executor.remove_node(peer_node);
        executor.remove_node(device_node);
        rclcpp::shutdown();
        std::cout << "zhixing_end_effector_test passed" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "zhixing_end_effector_test failed: " << error.what()
                  << std::endl;
        rclcpp::shutdown();
        return 1;
    }
}
