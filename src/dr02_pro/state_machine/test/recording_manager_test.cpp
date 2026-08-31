#include "recording_manager.hpp"

#include <H5Cpp.h>
#include <opencv2/imgcodecs.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <unistd.h>

using deep_robotics::dr02_pro::RecordingConfig;
using deep_robotics::dr02_pro::RecordingManager;
using deep_robotics::dr02_pro::RecordingSchema;
using deep_robotics::dr02_pro::RecordingStateFrame;

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

bool WaitForStatus(RecordingManager& manager,
                   RecordingManager::Status expected,
                   std::chrono::milliseconds timeout =
                       std::chrono::milliseconds(3000)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (manager.GetStatus() == expected) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
}

void RequireNear(double actual, double expected, const std::string& message) {
    Require(std::abs(actual - expected) < 1e-9, message);
}

void RequireDatasetShape(H5::H5File& file, const std::string& path,
                         const std::vector<hsize_t>& expected) {
    H5::DataSet dataset = file.openDataSet(path);
    H5::DataSpace space = dataset.getSpace();
    Require(
        static_cast<size_t>(space.getSimpleExtentNdims()) == expected.size(),
        path + " has the wrong rank");
    std::vector<hsize_t> actual(expected.size(), 0);
    space.getSimpleExtentDims(actual.data());
    Require(actual == expected, path + " has the wrong shape");
}

bool HasDeflateLevel(H5::DataSet& dataset, unsigned int expected_level) {
    H5::DSetCreatPropList properties = dataset.getCreatePlist();
    const int filter_count = H5Pget_nfilters(properties.getId());
    for (int index = 0; index < filter_count; ++index) {
        unsigned int flags = 0;
        unsigned int filter_config = 0;
        unsigned int values[8]{};
        size_t value_count = std::size(values);
        const H5Z_filter_t filter = H5Pget_filter2(
            properties.getId(), static_cast<unsigned int>(index), &flags,
            &value_count, values, 0, nullptr, &filter_config);
        if (filter == H5Z_FILTER_DEFLATE) {
            return value_count >= 1 && values[0] == expected_level;
        }
    }
    return false;
}

std::string ReadStringAttribute(H5::H5File& file,
                                const std::string& name) {
    std::string value;
    H5::Attribute attribute = file.openAttribute(name);
    attribute.read(attribute.getStrType(), value);
    return value;
}

void ValidateEpisode(const std::filesystem::path& path,
                     size_t state_dimension,
                     const std::string& state_layout,
                     const std::string& end_effector) {
    H5::H5File file(path.string(), H5F_ACC_RDONLY);
    int32_t fps = 0;
    double state_fps = 0.0;
    double camera_sync_tolerance_ms = 0.0;
    double max_camera_sync_skew_ms = 0.0;
    double max_abs_state_image_offset_ms = 0.0;
    uint64_t episode_len = 0;
    uint64_t recorded_state_dimension = 0;
    uint8_t depth_enabled = 1;
    file.openAttribute("fps").read(H5::PredType::NATIVE_INT32, &fps);
    file.openAttribute("state_fps").read(
        H5::PredType::NATIVE_DOUBLE, &state_fps);
    file.openAttribute("camera_sync_tolerance_ms").read(
        H5::PredType::NATIVE_DOUBLE, &camera_sync_tolerance_ms);
    file.openAttribute("max_camera_sync_skew_ms").read(
        H5::PredType::NATIVE_DOUBLE, &max_camera_sync_skew_ms);
    file.openAttribute("max_abs_state_image_offset_ms").read(
        H5::PredType::NATIVE_DOUBLE, &max_abs_state_image_offset_ms);
    file.openAttribute("episode_len").read(
        H5::PredType::NATIVE_UINT64, &episode_len);
    file.openAttribute("state_dimension").read(
        H5::PredType::NATIVE_UINT64, &recorded_state_dimension);
    file.openAttribute("depth_enabled").read(
        H5::PredType::NATIVE_UINT8, &depth_enabled);
    Require(fps == 30, "fps attribute is not 30");
    RequireNear(state_fps, 30.0, "state_fps attribute is not 30");
    RequireNear(camera_sync_tolerance_ms, 20.0,
                "camera sync tolerance attribute is not 20 ms");
    RequireNear(max_camera_sync_skew_ms, 10.0,
                "maximum camera skew attribute is wrong");
    RequireNear(max_abs_state_image_offset_ms, 5.0,
                "maximum state/image offset attribute is wrong");
    Require(episode_len == 2, "episode_len is not two");
    Require(recorded_state_dimension == state_dimension,
            "state_dimension attribute is wrong");
    Require(depth_enabled == 0, "depth_enabled attribute is not false");
    Require(
        ReadStringAttribute(file, "result") == "success",
        "result attribute is not success");
    Require(
        ReadStringAttribute(file, "state_image_match_policy") ==
            "nearest_received_publish_timestamp",
        "state/image matching policy is wrong");
    Require(
        ReadStringAttribute(file, "image_timestamp_source") ==
            "host_arrival_wall_time",
        "image timestamp source is wrong");
    Require(ReadStringAttribute(file, "state_layout") == state_layout,
            "state_layout attribute is wrong");
    Require(ReadStringAttribute(file, "end_effector") == end_effector,
            "end_effector attribute is wrong");

    H5::DataSet qpos = file.openDataSet("/observations/qpos");
    H5::DataSet actions = file.openDataSet("/actions");
    RequireDatasetShape(
        file, "/observations/qpos",
        {2, static_cast<hsize_t>(state_dimension)});
    RequireDatasetShape(
        file, "/actions", {2, static_cast<hsize_t>(state_dimension)});
    RequireDatasetShape(file, "/observations/control_mode", {2});
    RequireDatasetShape(file, "/observations/gate", {2});
    RequireDatasetShape(file, "/observations/timestamps", {2});
    RequireDatasetShape(file, "/observations/state_timestamps", {2});
    RequireDatasetShape(file, "/observations/state_time_offset_ms", {2});
    RequireDatasetShape(file, "/observations/camera_sync_skew_ms", {2});
    for (const std::string& camera :
         {"cam_d405_left", "cam_d405_right", "cam_d435"}) {
        RequireDatasetShape(
            file, "/observations/image_timestamps/" + camera, {2});
    }
    for (const std::string& camera :
         {"cam_d405_left", "cam_d405_right", "cam_d435"}) {
        RequireDatasetShape(
            file, "/observations/images/" + camera, {2, 500000});
        RequireDatasetShape(
            file, "/observations/images/" + camera + "_len", {2});
        H5::DataSet image =
            file.openDataSet("/observations/images/" + camera);
        Require(HasDeflateLevel(image, 1),
                "/observations/images/" + camera +
                    " is not using deflate level 1");
    }

    std::vector<double> qpos_values(2 * state_dimension, 0.0);
    std::vector<double> action_values(2 * state_dimension, 0.0);
    qpos.read(qpos_values.data(), H5::PredType::NATIVE_DOUBLE);
    actions.read(action_values.data(), H5::PredType::NATIVE_DOUBLE);
    for (size_t i = 0; i < state_dimension; ++i) {
        RequireNear(
            qpos_values[i], static_cast<double>(i) / 10.0,
            "qpos ordering is wrong");
        RequireNear(
            action_values[i], -static_cast<double>(i) / 10.0,
            "action ordering is wrong");
    }

    std::array<double, 2> control_mode{};
    std::array<double, 2> gate{};
    std::array<double, 2> image_timestamps{};
    std::array<double, 2> state_timestamps{};
    std::array<double, 2> offsets{};
    std::array<double, 2> camera_skews{};
    std::array<std::array<double, 2>, 3> per_camera_timestamps{};
    file.openDataSet("/observations/control_mode")
        .read(control_mode.data(), H5::PredType::NATIVE_DOUBLE);
    file.openDataSet("/observations/gate")
        .read(gate.data(), H5::PredType::NATIVE_DOUBLE);
    file.openDataSet("/observations/timestamps")
        .read(image_timestamps.data(), H5::PredType::NATIVE_DOUBLE);
    file.openDataSet("/observations/state_timestamps")
        .read(state_timestamps.data(), H5::PredType::NATIVE_DOUBLE);
    file.openDataSet("/observations/state_time_offset_ms")
        .read(offsets.data(), H5::PredType::NATIVE_DOUBLE);
    file.openDataSet("/observations/camera_sync_skew_ms")
        .read(camera_skews.data(), H5::PredType::NATIVE_DOUBLE);
    const std::array<std::string, 3> camera_names{
        "cam_d405_left", "cam_d405_right", "cam_d435"};
    for (size_t camera = 0; camera < camera_names.size(); ++camera) {
        file.openDataSet(
                "/observations/image_timestamps/" + camera_names[camera])
            .read(per_camera_timestamps[camera].data(),
                  H5::PredType::NATIVE_DOUBLE);
    }
    const std::array<std::array<double, 3>, 2> expected_image_timestamps{{
        {1000.0, 1005.0, 1010.0},
        {2032.0, 2030.0, 2031.0}}};
    for (size_t i = 0; i < 2; ++i) {
        RequireNear(control_mode[i], 1.0, "control_mode is not teleoperation");
        RequireNear(gate[i], 0.0, "gate is not zero");
        RequireNear(
            offsets[i], state_timestamps[i] - image_timestamps[i],
            "state/image time offset is inconsistent");
        double minimum_camera_timestamp =
            expected_image_timestamps[i][0];
        double maximum_camera_timestamp =
            expected_image_timestamps[i][0];
        for (size_t camera = 0; camera < camera_names.size(); ++camera) {
            RequireNear(per_camera_timestamps[camera][i],
                        expected_image_timestamps[i][camera],
                        "per-camera image timestamp is wrong");
            minimum_camera_timestamp = std::min(
                minimum_camera_timestamp,
                expected_image_timestamps[i][camera]);
            maximum_camera_timestamp = std::max(
                maximum_camera_timestamp,
                expected_image_timestamps[i][camera]);
        }
        RequireNear(image_timestamps[i], minimum_camera_timestamp,
                    "observation timestamp is not the earliest camera");
        RequireNear(camera_skews[i],
                    maximum_camera_timestamp - minimum_camera_timestamp,
                    "camera synchronization skew is wrong");
    }

    H5::DataSet image =
        file.openDataSet("/observations/images/cam_d405_left");
    H5::DataSet length =
        file.openDataSet("/observations/images/cam_d405_left_len");
    std::vector<uint64_t> jpeg_lengths(static_cast<size_t>(episode_len), 0);
    length.read(jpeg_lengths.data(), H5::PredType::NATIVE_UINT64);
    const uint64_t jpeg_length = jpeg_lengths.front();
    Require(
        jpeg_length > 0 && jpeg_length <= 500000,
        "invalid JPEG length");
    std::vector<uint8_t> padded(
        static_cast<size_t>(episode_len) * 500000);
    image.read(padded.data(), H5::PredType::NATIVE_UINT8);
    padded.resize(static_cast<size_t>(jpeg_length));
    const cv::Mat decoded = cv::imdecode(padded, cv::IMREAD_COLOR);
    Require(
        !decoded.empty() && decoded.cols == 640 && decoded.rows == 480,
        "stored JPEG cannot be decoded");
}

}  // namespace

int main() {
    const std::filesystem::path root =
        std::filesystem::path("/tmp") /
        ("dr02_recording_manager_test_" + std::to_string(getpid()));
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);
    {
        std::ofstream stale(root / "episode_0.hdf5.tmp");
        stale << "stale";
    }

    try {
        RecordingManager manager(RecordingConfig{true, root, {}});
        Require(manager.Start(), "manager failed to start");
        Require(
            !std::filesystem::exists(root / "episode_0.hdf5.tmp"),
            "stale temporary file was not removed");
        Require(
            !manager.RequestStartEpisode(),
            "episode started without three ready cameras");
        manager.SetSyntheticCamerasReadyForTesting();
        Require(
            WaitForStatus(manager, RecordingManager::Status::Idle),
            "manager did not become idle");

        Require(manager.RequestStartEpisode(), "episode start rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Recording),
            "manager did not start recording");

        RecordingStateFrame state;
        state.timestamp_ms = 1005.0;
        state.qpos.resize(14);
        state.action.resize(14);
        for (size_t i = 0; i < state.qpos.size(); ++i) {
            state.qpos[i] = static_cast<double>(i) / 10.0;
            state.action[i] = -static_cast<double>(i) / 10.0;
        }
        manager.AddStateFrame(state);
        std::vector<uint8_t> rgb(640 * 480 * 3, 0);
        for (size_t i = 0; i < rgb.size(); i += 3) {
            rgb[i] = 20;
            rgb[i + 1] = 80;
            rgb[i + 2] = 160;
        }
        manager.AddCameraFrameForTesting(0, 1000.0, rgb);
        manager.AddCameraFrameForTesting(1, 1005.0, rgb);
        manager.AddCameraFrameForTesting(2, 1010.0, rgb);

        // A 31 ms skew drops the earliest frame. Supplying a replacement
        // within the 20 ms window must then produce exactly one more row.
        state.timestamp_ms = 2031.0;
        manager.AddStateFrame(state);
        manager.AddCameraFrameForTesting(0, 2000.0, rgb);
        manager.AddCameraFrameForTesting(1, 2030.0, rgb);
        manager.AddCameraFrameForTesting(2, 2031.0, rgb);
        manager.AddCameraFrameForTesting(0, 2032.0, rgb);
        Require(manager.RequestFinishEpisode(), "episode finish rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Idle),
            "manager did not finish recording");
        const auto episode = root / "episode_0.hdf5";
        Require(std::filesystem::exists(episode), "final episode is missing");
        Require(
            !std::filesystem::exists(root / "episode_0.hdf5.tmp"),
            "temporary episode remains after finish");
        ValidateEpisode(episode, 14, "left_arm_7,right_arm_7", "none");

        Require(manager.RequestStartEpisode(), "second episode start rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Recording),
            "second episode did not start");
        Require(
            manager.RequestAbortEpisode("test abort"),
            "episode abort rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Idle),
            "manager did not abort recording");
        Require(
            !std::filesystem::exists(root / "episode_1.hdf5") &&
                !std::filesystem::exists(root / "episode_1.hdf5.tmp"),
            "aborted episode was retained");

        Require(
            manager.RequestStartEpisode(),
            "zero-frame episode start rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Recording),
            "zero-frame episode did not start");
        Require(
            manager.RequestFinishEpisode(),
            "zero-frame episode finish rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Idle),
            "zero-frame episode did not finish");
        Require(
            !std::filesystem::exists(root / "episode_1.hdf5") &&
                !std::filesystem::exists(root / "episode_1.hdf5.tmp"),
            "zero-frame episode was retained");

        Require(
            manager.RequestDeleteLastEpisode(),
            "delete last episode rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Idle),
            "manager did not finish deletion");
        Require(
            !std::filesystem::exists(episode),
            "latest episode was not deleted");

        Require(
            manager.RequestStartEpisode(),
            "reused episode start rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Recording),
            "reused episode did not start");
        Require(
            std::filesystem::exists(root / "episode_0.hdf5.tmp"),
            "deleted episode number was not reused");
        Require(
            manager.RequestAbortEpisode("number reuse test"),
            "reused episode abort rejected");
        Require(
            WaitForStatus(manager, RecordingManager::Status::Idle),
            "reused episode did not abort");
        manager.Shutdown();

        const std::filesystem::path root_with_ee = root / "with_ee";
        RecordingSchema schema;
        schema.state_names.push_back("left_zhixing_gripper");
        schema.state_names.push_back("right_zhixing_gripper");
        schema.layout =
            "left_arm_7,right_arm_7,left_end_effector_1,right_end_effector_1";
        schema.end_effector = "zhixing";
        RecordingManager manager_with_ee(
            RecordingConfig{true, root_with_ee, schema});
        Require(manager_with_ee.Start(),
                "16-dimensional manager failed to start");
        manager_with_ee.SetSyntheticCamerasReadyForTesting();
        Require(WaitForStatus(manager_with_ee, RecordingManager::Status::Idle),
                "16-dimensional manager did not become idle");
        Require(manager_with_ee.RequestStartEpisode(),
                "16-dimensional episode start rejected");
        Require(WaitForStatus(manager_with_ee,
                              RecordingManager::Status::Recording),
                "16-dimensional episode did not start");
        RecordingStateFrame state_with_ee;
        state_with_ee.timestamp_ms = 1005.0;
        state_with_ee.qpos.resize(16);
        state_with_ee.action.resize(16);
        for (size_t i = 0; i < state_with_ee.qpos.size(); ++i) {
            state_with_ee.qpos[i] = static_cast<double>(i) / 10.0;
            state_with_ee.action[i] = -static_cast<double>(i) / 10.0;
        }
        manager_with_ee.AddStateFrame(state_with_ee);
        manager_with_ee.AddCameraFrameForTesting(0, 1000.0, rgb);
        manager_with_ee.AddCameraFrameForTesting(1, 1005.0, rgb);
        manager_with_ee.AddCameraFrameForTesting(2, 1010.0, rgb);
        state_with_ee.timestamp_ms = 2031.0;
        manager_with_ee.AddStateFrame(state_with_ee);
        manager_with_ee.AddCameraFrameForTesting(0, 2000.0, rgb);
        manager_with_ee.AddCameraFrameForTesting(1, 2030.0, rgb);
        manager_with_ee.AddCameraFrameForTesting(2, 2031.0, rgb);
        manager_with_ee.AddCameraFrameForTesting(0, 2032.0, rgb);
        Require(manager_with_ee.RequestFinishEpisode(),
                "16-dimensional episode finish rejected");
        Require(WaitForStatus(manager_with_ee, RecordingManager::Status::Idle),
                "16-dimensional episode did not finish");
        ValidateEpisode(
            root_with_ee / "episode_0.hdf5", 16, schema.layout, "zhixing");

        Require(manager_with_ee.RequestStartEpisode(),
                "dimension-mismatch episode start rejected");
        Require(WaitForStatus(manager_with_ee,
                              RecordingManager::Status::Recording),
                "dimension-mismatch episode did not start");
        RecordingStateFrame wrong_dimension;
        wrong_dimension.timestamp_ms = 3000.0;
        wrong_dimension.qpos.resize(14);
        wrong_dimension.action.resize(14);
        manager_with_ee.AddStateFrame(wrong_dimension);
        Require(WaitForStatus(manager_with_ee, RecordingManager::Status::Idle),
                "dimension mismatch did not abort the episode");
        Require(!std::filesystem::exists(
                    root_with_ee / "episode_1.hdf5.tmp"),
                "dimension-mismatch temporary episode was retained");
        manager_with_ee.Shutdown();
        std::filesystem::remove_all(root);
    } catch (const std::exception& error) {
        std::cerr << "recording_manager_test failed: " << error.what()
                  << std::endl;
        std::filesystem::remove_all(root);
        return 1;
    }

    std::cout << "recording_manager_test passed" << std::endl;
    return 0;
}
