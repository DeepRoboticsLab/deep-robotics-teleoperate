/**
 * @file pico_data_receiver.cpp
 * @brief Pico PXREA data receiver and parsed state cache.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#include "pico_data_receiver.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <dlfcn.h>
#include <iostream>
#include <mutex>
#include <string>
#include <vector>

#include "PXREARobotSDK.h"
#include "json.hpp"

using json = nlohmann::json;

namespace deep_robotics::dr02_pro {
namespace {

constexpr double kHeadPoseEpsilon = 1e-6;

#ifndef DR02_PRO_PXREA_LIB
#define DR02_PRO_PXREA_LIB "libPXREARobotSDK.so"
#endif

using PXREAInitFn = int (*)(void*, pfPXREAClientCallback, unsigned);
using PXREADeinitFn = int (*)();

std::mutex g_pico_mutex;
PicoState g_pico_state;
std::atomic<bool> g_pico_connected{false};
std::mutex g_pxrea_mutex;
void* g_pxrea_handle = nullptr;
PXREAInitFn g_pxrea_init = nullptr;
PXREADeinitFn g_pxrea_deinit = nullptr;

uint64_t SteadyNowNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::steady_clock::now().time_since_epoch())
                                     .count());
}

bool LoadPxreaSdk() {
    std::lock_guard<std::mutex> lock(g_pxrea_mutex);
    if (g_pxrea_init != nullptr && g_pxrea_deinit != nullptr) {
        return true;
    }

    g_pxrea_handle = dlopen(DR02_PRO_PXREA_LIB, RTLD_NOW | RTLD_LOCAL);
    if (g_pxrea_handle == nullptr) {
        std::cerr << "[PicoTeleoperation] dlopen PXREA failed: " << dlerror() << std::endl;
        return false;
    }

    dlerror();
    g_pxrea_init = reinterpret_cast<PXREAInitFn>(dlsym(g_pxrea_handle, "PXREAInit"));
    const char* init_error = dlerror();
    dlerror();
    g_pxrea_deinit = reinterpret_cast<PXREADeinitFn>(dlsym(g_pxrea_handle, "PXREADeinit"));
    const char* deinit_error = dlerror();
    if (init_error != nullptr || deinit_error != nullptr || g_pxrea_init == nullptr || g_pxrea_deinit == nullptr) {
        std::cerr << "[PicoTeleoperation] dlsym PXREA failed"
                  << (init_error != nullptr ? init_error : "")
                  << (deinit_error != nullptr ? deinit_error : "") << std::endl;
        dlclose(g_pxrea_handle);
        g_pxrea_handle = nullptr;
        g_pxrea_init = nullptr;
        g_pxrea_deinit = nullptr;
        return false;
    }

    return true;
}


std::vector<double> SplitPose(const std::string& pose) {
    std::vector<double> values;
    values.reserve(7);
    size_t start = 0;
    while (start <= pose.size()) {
        const size_t comma = pose.find(',', start);
        const std::string token = pose.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
        if (!token.empty()) {
            values.push_back(std::stod(token));
        }
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    return values;
}

PicoControllerState ParseController(const json& value) {
    PicoControllerState out;
    const auto pose_values = SplitPose(value.at("pose").get<std::string>());
    if (pose_values.size() >= 7) {
        out.pos = {pose_values[0], pose_values[1], pose_values[2]};
        out.quat = {pose_values[3], pose_values[4], pose_values[5], pose_values[6]};
    }
    out.grip = static_cast<float>(value.value("grip", 0.0));
    out.trigger = static_cast<float>(value.value("trigger", 0.0));
    out.primary = value.value("primaryButton", false);
    out.secondary = value.value("secondaryButton", false);
    out.axis_click = value.value("axisClick", false);
    out.axis = {static_cast<float>(value.value("axisX", 0.0)), static_cast<float>(value.value("axisY", 0.0))};
    out.valid = true;
    return out;
}

bool IsValidHeadPose(const PicoHeadState& head) {
    double sum_abs = 0.0;
    double q_norm_sq = 0.0;
    for (double v : head.pos) {
        if (!std::isfinite(v)) {
            return false;
        }
        sum_abs += std::abs(v);
    }
    for (double v : head.quat) {
        if (!std::isfinite(v)) {
            return false;
        }
        sum_abs += std::abs(v);
        q_norm_sq += v * v;
    }
    return q_norm_sq > kHeadPoseEpsilon && sum_abs > kHeadPoseEpsilon;
}

bool FillHeadPoseFromValues(const std::vector<double>& values, PicoHeadState& out) {
    if (values.size() < 7) {
        return false;
    }
    PicoHeadState candidate;
    candidate.pos = {values[0], values[1], values[2]};
    candidate.quat = {values[3], values[4], values[5], values[6]};
    candidate.valid = IsValidHeadPose(candidate);
    if (!candidate.valid) {
        return false;
    }
    out = candidate;
    return true;
}

bool TryParseHeadPose(const json& value, PicoHeadState& out) {
    try {
        if (value.is_string()) {
            return FillHeadPoseFromValues(SplitPose(value.get<std::string>()), out);
        }
        if (value.is_array()) {
            std::vector<double> values;
            values.reserve(value.size());
            for (const auto& item : value) {
                if (item.is_number()) {
                    values.push_back(item.get<double>());
                } else if (item.is_string()) {
                    values.push_back(std::stod(item.get<std::string>()));
                }
            }
            return FillHeadPoseFromValues(values, out);
        }
        if (!value.is_object()) {
            return false;
        }

        const std::array<std::string, 6> pose_keys{
            "pose", "headPose", "head_pose", "headsetPose", "headset_pose", "hmdPose"};
        for (const auto& key : pose_keys) {
            if (value.contains(key) && TryParseHeadPose(value.at(key), out)) {
                return true;
            }
        }

        const std::array<std::string, 8> object_keys{
            "Head", "head", "HMD", "hmd", "Headset", "headset", "XRHead", "xrHead"};
        for (const auto& key : object_keys) {
            if (value.contains(key) && TryParseHeadPose(value.at(key), out)) {
                return true;
            }
        }
    } catch (const std::exception&) {
        return false;
    }
    return false;
}

void OnPicoCallback(void*, PXREAClientCallbackType type, int status, void* user_data) {
    switch (type) {
        case PXREAServerConnect:
            g_pico_connected.store(true);
            std::cout << "[PicoTeleoperation] PXREA server connected" << std::endl;
            break;
        case PXREAServerDisconnect:
            g_pico_connected.store(false);
            std::cout << "[PicoTeleoperation] PXREA server disconnected" << std::endl;
            break;
        case PXREADeviceFind:
            std::cout << "[PicoTeleoperation] device found: " << static_cast<const char*>(user_data) << std::endl;
            break;
        case PXREADeviceMissing:
            std::cout << "[PicoTeleoperation] device missing: " << static_cast<const char*>(user_data) << std::endl;
            break;
        case PXREADeviceConnect:
            std::cout << "[PicoTeleoperation] device connected: " << static_cast<const char*>(user_data)
                      << " status=" << status << std::endl;
            break;
        case PXREADeviceStateJson: {
            const auto& dsj = *static_cast<PXREADevStateJson*>(user_data);
            try {
                const json packet = json::parse(dsj.stateJson);
                if (!packet.contains("value")) {
                    return;
                }
                const json value = json::parse(packet.at("value").get<std::string>());
                std::lock_guard<std::mutex> lock(g_pico_mutex);
                const bool has_controllers = value.contains("Controller");
                bool has_left = false;
                bool has_right = false;
                if (value.contains("Controller")) {
                    const auto& controllers = value.at("Controller");
                    if (controllers.contains("left")) {
                        g_pico_state.left = ParseController(controllers.at("left"));
                        has_left = true;
                    }
                    if (controllers.contains("right")) {
                        g_pico_state.right = ParseController(controllers.at("right"));
                        has_right = true;
                    }
                }
                PicoHeadState head;
                if (TryParseHeadPose(value, head)) {
                    g_pico_state.head = head;
                }
                ++g_pico_state.state_sequence;
                ++g_pico_state.valid_packet_count;
                g_pico_state.received_steady_ns = SteadyNowNs();
                if (g_pico_state.valid_packet_count == 1) {
                    std::cout << "[PicoTeleoperation] first state JSON from " << dsj.devID
                              << ": Controller=" << has_controllers << " left=" << has_left
                              << " right=" << has_right << std::endl;
                }
            } catch (const std::exception& e) {
                {
                    std::lock_guard<std::mutex> lock(g_pico_mutex);
                    ++g_pico_state.parse_error_count;
                }
                std::cerr << "[PicoTeleoperation] parse Pico json failed: " << e.what() << std::endl;
            }
            break;
        }
        default:
            break;
    }
}

}  // namespace

PicoDataReceiver::~PicoDataReceiver() {
    Stop();
}

int PicoDataReceiver::Start() {
    if (started_) {
        return 0;
    }
    if (!LoadPxreaSdk()) {
        return -1;
    }

    {
        std::lock_guard<std::mutex> lock(g_pico_mutex);
        g_pico_state = PicoState{};
    }
    g_pico_connected.store(false);

    const int rc = g_pxrea_init(nullptr, &OnPicoCallback, PXREAFullMask);
    started_ = (rc == 0);
    std::cout << "[PicoTeleoperation] PXREAInit returned " << rc << std::endl;
    return rc;
}

void PicoDataReceiver::Stop() {
    if (!started_) {
        return;
    }
    if (g_pxrea_deinit != nullptr) {
        g_pxrea_deinit();
    }
    started_ = false;
}

PicoState PicoDataReceiver::Snapshot() const {
    std::lock_guard<std::mutex> lock(g_pico_mutex);
    return g_pico_state;
}

bool PicoDataReceiver::IsConnected() const {
    return g_pico_connected.load();
}

}  // namespace deep_robotics::dr02_pro
