/**
 * @file recording_manager.hpp
 * @brief Optional three-camera RGB/HDF5 data recorder.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace deep_robotics::dr02_pro {

struct RecordingSchema {
    std::vector<std::string> state_names{
        "left_arm_joint_1", "left_arm_joint_2", "left_arm_joint_3",
        "left_arm_joint_4", "left_arm_joint_5", "left_arm_joint_6",
        "left_arm_joint_7", "right_arm_joint_1", "right_arm_joint_2",
        "right_arm_joint_3", "right_arm_joint_4", "right_arm_joint_5",
        "right_arm_joint_6", "right_arm_joint_7"};
    std::string layout{"left_arm_7,right_arm_7"};
    std::string end_effector{"none"};

    size_t Dimension() const { return state_names.size(); }
};

struct RecordingConfig {
    bool enabled{false};
    std::filesystem::path dataset_root;
    RecordingSchema schema;
};

struct RecordingStateFrame {
    double timestamp_ms{0.0};
    std::vector<double> qpos;
    std::vector<double> action;
};

class RecordingManager {
public:
    enum class Status {
        Disabled,
        Unavailable,
        Idle,
        Starting,
        Recording,
        Finalizing,
        Aborting,
        Deleting,
    };

    explicit RecordingManager(RecordingConfig config);
    ~RecordingManager();

    RecordingManager(const RecordingManager&) = delete;
    RecordingManager& operator=(const RecordingManager&) = delete;

    bool Start();
    void Shutdown();

    bool RequestStartEpisode();
    bool RequestFinishEpisode();
    bool RequestAbortEpisode(const std::string& reason);
    bool RequestDeleteLastEpisode();

    void AddStateFrame(const RecordingStateFrame& frame);

    Status GetStatus() const;
    bool IsEnabled() const;
    bool IsRecording() const;

#ifdef DR02_PRO_RECORDING_TEST_API
    void SetSyntheticCamerasReadyForTesting();
    void AddCameraFrameForTesting(
        size_t camera_index, double timestamp_ms,
        const std::vector<uint8_t>& rgb);
#endif

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace deep_robotics::dr02_pro
