/**
 * @file pico_data_receiver.hpp
 * @brief Pico PXREA data receiver and parsed state cache.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#pragma once

#include <array>
#include <cstdint>

namespace deep_robotics::dr02_pro {

struct PicoControllerState {
    bool valid{false};
    bool primary{false};
    bool secondary{false};
    bool axis_click{false};
    float grip{0.0f};
    float trigger{0.0f};
    std::array<float, 2> axis{0.0f, 0.0f};
    std::array<double, 3> pos{0.0, 0.0, 0.0};
    std::array<double, 4> quat{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
};

struct PicoHeadState {
    bool valid{false};
    std::array<double, 3> pos{0.0, 0.0, 0.0};
    std::array<double, 4> quat{0.0, 0.0, 0.0, 1.0};  // x, y, z, w
};

struct PicoState {
    PicoControllerState left;
    PicoControllerState right;
    PicoHeadState head;
    // Monotonic local receive time, not a sender timestamp. It is therefore
    // suitable for detecting a stale Pico stream but not for network latency.
    uint64_t received_steady_ns{0};
    uint64_t valid_packet_count{0};
    uint64_t parse_error_count{0};
    uint64_t state_sequence{0};
};

class PicoDataReceiver {
public:
    PicoDataReceiver() = default;
    ~PicoDataReceiver();

    PicoDataReceiver(const PicoDataReceiver&) = delete;
    PicoDataReceiver& operator=(const PicoDataReceiver&) = delete;

    int Start();
    void Stop();
    PicoState Snapshot() const;
    bool IsConnected() const;

private:
    bool started_{false};
};

}  // namespace deep_robotics::dr02_pro
