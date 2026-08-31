/**
 * @file recording_manager.cpp
 * @brief Three-camera synchronization and asynchronous HDF5 episode writer.
 */
#include "recording_manager.hpp"

#include <H5Cpp.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#ifdef DR02_PRO_HAS_REALSENSE
#include <librealsense2/rs.hpp>
#endif

namespace deep_robotics::dr02_pro {
namespace {

constexpr int kWidth = 640;
constexpr int kHeight = 480;
constexpr int kChannels = 3;
constexpr int kFps = 30;
constexpr int kJpegQuality = 95;
constexpr size_t kMaxJpegBytes = 500000;
constexpr size_t kMaxCameraFrames = 5;
constexpr size_t kMaxStateFrames = 100;
constexpr size_t kMaxPendingSamples = 120;
constexpr double kSyncToleranceMs = 20.0;
constexpr double kCameraStallMs = 2000.0;
constexpr double kCameraRetryMs = 2000.0;

constexpr std::array<const char*, 3> kCameraNames{
    "d405_left", "d405_right", "d435"};
constexpr std::array<const char*, 3> kCameraSerials{
    "260322270318", "260322274028", "346122072375"};

double WallNowMs() {
    return static_cast<double>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

bool IsActiveStatus(RecordingManager::Status status) {
    return status == RecordingManager::Status::Starting ||
           status == RecordingManager::Status::Recording ||
           status == RecordingManager::Status::Finalizing ||
           status == RecordingManager::Status::Aborting;
}

int DetermineNextEpisodeIndex(const std::filesystem::path& root) {
    int max_index = -1;
    const std::regex pattern(R"(^episode_([0-9]+)\.hdf5$)");
    if (!std::filesystem::exists(root)) {
        return 0;
    }
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (!entry.is_regular_file()) {
            continue;
        }
        std::smatch match;
        const std::string name = entry.path().filename().string();
        if (std::regex_match(name, match, pattern)) {
            max_index = std::max(max_index, std::stoi(match[1].str()));
        }
    }
    return max_index + 1;
}

void CleanupStaleTemporaryFiles(const std::filesystem::path& root) {
    const std::regex pattern(R"(^episode_[0-9]+\.hdf5\.tmp$)");
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (!entry.is_regular_file() ||
            !std::regex_match(entry.path().filename().string(), pattern)) {
            continue;
        }
        std::error_code error;
        if (std::filesystem::remove(entry.path(), error)) {
            std::cout << "[Recording] removed stale temporary file: "
                      << entry.path() << std::endl;
        } else {
            std::cerr << "[Recording] failed to remove stale temporary file "
                      << entry.path() << ": " << error.message() << std::endl;
        }
    }
}

class EpisodeWriter {
public:
    EpisodeWriter() {
        H5::Exception::dontPrint();
    }

    ~EpisodeWriter() {
        Abort();
    }

    void Open(const std::filesystem::path& root, int episode_index,
              const RecordingSchema& schema) {
        Abort();
        if (schema.Dimension() == 0) {
            throw std::runtime_error("recording schema has no state fields");
        }
        state_dimension_ = static_cast<hsize_t>(schema.Dimension());
        episode_index_ = episode_index;
        temporary_path_ =
            root / ("episode_" + std::to_string(episode_index) + ".hdf5.tmp");
        final_path_ =
            root / ("episode_" + std::to_string(episode_index) + ".hdf5");

        file_ = std::make_unique<H5::H5File>(
            temporary_path_.string(), H5F_ACC_TRUNC);
        WriteAttribute(*file_, "fps", static_cast<int32_t>(kFps));
        WriteAttribute(*file_, "state_fps", static_cast<double>(kFps));
        WriteAttribute(*file_, "state_dimension",
                       static_cast<uint64_t>(state_dimension_));
        WriteStringAttribute(*file_, "state_layout", schema.layout);
        WriteStringAttribute(*file_, "end_effector", schema.end_effector);
        WriteStringAttribute(
            *file_, "state_image_match_policy",
            "nearest_received_publish_timestamp");
        WriteStringAttribute(
            *file_, "image_timestamp_source", "host_arrival_wall_time");
        WriteAttribute(*file_, "camera_sync_tolerance_ms", kSyncToleranceMs);
        WriteAttribute(*file_, "depth_enabled", static_cast<uint8_t>(0));
#ifdef USE_SIMULATION
        WriteAttribute(*file_, "sim", static_cast<uint8_t>(1));
#else
        WriteAttribute(*file_, "sim", static_cast<uint8_t>(0));
#endif

        file_->createGroup("/observations");
        file_->createGroup("/observations/images");
        file_->createGroup("/observations/image_timestamps");

        qpos_ = CreateMatrixDataset("/observations/qpos", state_dimension_);
        actions_ = CreateMatrixDataset("/actions", state_dimension_);
        control_mode_ = CreateScalarDataset("/observations/control_mode");
        gate_ = CreateScalarDataset("/observations/gate");
        image_timestamp_ = CreateScalarDataset("/observations/timestamps");
        state_timestamp_ =
            CreateScalarDataset("/observations/state_timestamps");
        state_time_offset_ =
            CreateScalarDataset("/observations/state_time_offset_ms");
        camera_sync_skew_ =
            CreateScalarDataset("/observations/camera_sync_skew_ms");
        image_timestamp_left_ = CreateScalarDataset(
            "/observations/image_timestamps/cam_d405_left");
        image_timestamp_right_ = CreateScalarDataset(
            "/observations/image_timestamps/cam_d405_right");
        image_timestamp_head_ = CreateScalarDataset(
            "/observations/image_timestamps/cam_d435");

        image_left_ =
            CreateImageDataset("/observations/images/cam_d405_left");
        image_right_ =
            CreateImageDataset("/observations/images/cam_d405_right");
        image_head_ =
            CreateImageDataset("/observations/images/cam_d435");
        image_left_len_ =
            CreateLengthDataset("/observations/images/cam_d405_left_len");
        image_right_len_ =
            CreateLengthDataset("/observations/images/cam_d405_right_len");
        image_head_len_ =
            CreateLengthDataset("/observations/images/cam_d435_len");

        WriteStringAttribute(
            control_mode_, "meaning",
            "0=policy, 1=teleop_intervention, 2=policy_release");
        frame_count_ = 0;
        max_camera_sync_skew_ms_ = 0.0;
        max_abs_state_image_offset_ms_ = 0.0;
        std::cout << "[Recording] opened temporary episode: "
                  << temporary_path_ << std::endl;
    }

    void Append(const RecordingStateFrame& state, double image_timestamp_ms,
                const std::array<double, 3>& image_timestamps_ms,
                double camera_sync_skew_ms,
                const std::array<std::vector<uint8_t>, 3>& images) {
        if (!file_) {
            throw std::runtime_error("episode file is not open");
        }

        std::array<std::vector<uint8_t>, 3> jpeg;
        for (size_t i = 0; i < images.size(); ++i) {
            jpeg[i] = EncodeJpeg(images[i]);
        }

        const hsize_t row = static_cast<hsize_t>(frame_count_);
        if (state.qpos.size() != state_dimension_ ||
            state.action.size() != state_dimension_) {
            throw std::runtime_error("recording state dimension mismatch");
        }
        AppendMatrixRow(qpos_, row, state.qpos.data(), state_dimension_);
        AppendMatrixRow(actions_, row, state.action.data(), state_dimension_);
        AppendScalar(control_mode_, row, 1.0);
        AppendScalar(gate_, row, 0.0);
        AppendScalar(image_timestamp_, row, image_timestamp_ms);
        AppendScalar(state_timestamp_, row, state.timestamp_ms);
        const double state_image_offset_ms =
            state.timestamp_ms - image_timestamp_ms;
        AppendScalar(state_time_offset_, row, state_image_offset_ms);
        AppendScalar(camera_sync_skew_, row, camera_sync_skew_ms);
        AppendScalar(image_timestamp_left_, row, image_timestamps_ms[0]);
        AppendScalar(image_timestamp_right_, row, image_timestamps_ms[1]);
        AppendScalar(image_timestamp_head_, row, image_timestamps_ms[2]);
        max_camera_sync_skew_ms_ =
            std::max(max_camera_sync_skew_ms_, camera_sync_skew_ms);
        max_abs_state_image_offset_ms_ = std::max(
            max_abs_state_image_offset_ms_, std::abs(state_image_offset_ms));
        AppendImage(image_left_, image_left_len_, row, jpeg[0]);
        AppendImage(image_right_, image_right_len_, row, jpeg[1]);
        AppendImage(image_head_, image_head_len_, row, jpeg[2]);
        ++frame_count_;
    }

    bool Finish() {
        if (!file_ || frame_count_ == 0) {
            Abort();
            return false;
        }

        WriteAttribute(
            *file_, "episode_len", static_cast<uint64_t>(frame_count_));
        WriteStringAttribute(*file_, "result", "success");
        WriteAttribute(*file_, "max_camera_sync_skew_ms",
                       max_camera_sync_skew_ms_);
        WriteAttribute(*file_, "max_abs_state_image_offset_ms",
                       max_abs_state_image_offset_ms_);
        file_->flush(H5F_SCOPE_GLOBAL);
        CloseHandles();

        std::error_code error;
        std::filesystem::rename(temporary_path_, final_path_, error);
        if (error) {
            std::filesystem::remove(temporary_path_);
            throw std::runtime_error(
                "rename episode failed: " + error.message());
        }
        std::cout << "[Recording] saved " << final_path_
                  << ", frames=" << frame_count_ << std::endl;
        frame_count_ = 0;
        return true;
    }

    void Abort() {
        CloseHandles();
        if (!temporary_path_.empty()) {
            std::error_code error;
            const bool removed =
                std::filesystem::remove(temporary_path_, error);
            if (removed) {
                std::cout << "[Recording] removed incomplete episode: "
                          << temporary_path_ << std::endl;
            } else if (error && std::filesystem::exists(temporary_path_)) {
                std::cerr << "[Recording] failed to remove incomplete episode "
                          << temporary_path_ << ": " << error.message()
                          << std::endl;
            }
        }
        temporary_path_.clear();
        final_path_.clear();
        frame_count_ = 0;
        max_camera_sync_skew_ms_ = 0.0;
        max_abs_state_image_offset_ms_ = 0.0;
        state_dimension_ = 0;
    }

    bool IsOpen() const {
        return static_cast<bool>(file_);
    }

private:
    template <typename Object, typename Value>
    static void WriteAttribute(Object& object, const std::string& name,
                               const Value& value) {
        H5::DataSpace scalar(H5S_SCALAR);
        H5::Attribute attribute =
            object.createAttribute(name, NativeType<Value>(), scalar);
        attribute.write(NativeType<Value>(), &value);
    }

    template <typename Value>
    static const H5::PredType& NativeType();

    template <typename Object>
    static void WriteStringAttribute(Object& object, const std::string& name,
                                     const std::string& value) {
        H5::DataSpace scalar(H5S_SCALAR);
        H5::StrType type(H5::PredType::C_S1, H5T_VARIABLE);
        H5::Attribute attribute = object.createAttribute(name, type, scalar);
        attribute.write(type, value);
    }

    H5::DataSet CreateMatrixDataset(const std::string& path, hsize_t width) {
        const hsize_t dims[2]{0, width};
        const hsize_t max_dims[2]{H5S_UNLIMITED, width};
        const hsize_t chunk[2]{1, width};
        H5::DataSpace space(2, dims, max_dims);
        H5::DSetCreatPropList properties;
        properties.setChunk(2, chunk);
        properties.setDeflate(1);
        H5::DataSet dataset =
            file_->createDataSet(path, H5::PredType::NATIVE_DOUBLE, space,
                                 properties);
        WriteStringAttribute(dataset, "layout", "NF");
        WriteStringAttribute(dataset, "dtype", "float64");
        return dataset;
    }

    H5::DataSet CreateScalarDataset(const std::string& path) {
        const hsize_t dims[1]{0};
        const hsize_t max_dims[1]{H5S_UNLIMITED};
        const hsize_t chunk[1]{1};
        H5::DataSpace space(1, dims, max_dims);
        H5::DSetCreatPropList properties;
        properties.setChunk(1, chunk);
        properties.setDeflate(1);
        H5::DataSet dataset =
            file_->createDataSet(path, H5::PredType::NATIVE_DOUBLE, space,
                                 properties);
        WriteStringAttribute(dataset, "layout", "N");
        WriteStringAttribute(dataset, "dtype", "float64");
        return dataset;
    }

    H5::DataSet CreateImageDataset(const std::string& path) {
        const hsize_t dims[2]{0, kMaxJpegBytes};
        const hsize_t max_dims[2]{H5S_UNLIMITED, kMaxJpegBytes};
        const hsize_t chunk[2]{1, kMaxJpegBytes};
        H5::DataSpace space(2, dims, max_dims);
        H5::DSetCreatPropList properties;
        properties.setChunk(2, chunk);
        // JPEG payloads are stored in fixed zero-padded rows. Level-1
        // deflate removes the padding while preserving the existing schema.
        properties.setDeflate(1);
        H5::DataSet dataset =
            file_->createDataSet(path, H5::PredType::NATIVE_UINT8, space,
                                 properties);
        WriteStringAttribute(dataset, "encoding", "jpeg");
        WriteAttribute(dataset, "jpeg_quality",
                       static_cast<int32_t>(kJpegQuality));
        WriteAttribute(dataset, "original_height",
                       static_cast<int32_t>(kHeight));
        WriteAttribute(dataset, "original_width",
                       static_cast<int32_t>(kWidth));
        WriteAttribute(dataset, "original_channels",
                       static_cast<int32_t>(kChannels));
        WriteStringAttribute(dataset, "color", "RGB");
        WriteStringAttribute(dataset, "dtype", "uint8");
        return dataset;
    }

    H5::DataSet CreateLengthDataset(const std::string& path) {
        const hsize_t dims[1]{0};
        const hsize_t max_dims[1]{H5S_UNLIMITED};
        const hsize_t chunk[1]{1};
        H5::DataSpace space(1, dims, max_dims);
        H5::DSetCreatPropList properties;
        properties.setChunk(1, chunk);
        return file_->createDataSet(
            path, H5::PredType::NATIVE_UINT64, space, properties);
    }

    static std::vector<uint8_t> EncodeJpeg(
        const std::vector<uint8_t>& rgb_data) {
        const size_t expected =
            static_cast<size_t>(kWidth * kHeight * kChannels);
        if (rgb_data.size() != expected) {
            throw std::runtime_error("invalid RGB frame size");
        }
        cv::Mat rgb(
            kHeight, kWidth, CV_8UC3,
            const_cast<uint8_t*>(rgb_data.data()));
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        std::vector<uint8_t> encoded;
        const std::vector<int> parameters{
            cv::IMWRITE_JPEG_QUALITY, kJpegQuality};
        if (!cv::imencode(".jpg", bgr, encoded, parameters)) {
            throw std::runtime_error("JPEG encoding failed");
        }
        if (encoded.size() > kMaxJpegBytes) {
            throw std::runtime_error("JPEG frame exceeds fixed HDF5 row");
        }
        return encoded;
    }

    static void AppendMatrixRow(H5::DataSet& dataset, hsize_t row,
                                const double* data, hsize_t width) {
        const hsize_t new_size[2]{row + 1, width};
        dataset.extend(new_size);
        H5::DataSpace file_space = dataset.getSpace();
        const hsize_t offset[2]{row, 0};
        const hsize_t count[2]{1, width};
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        H5::DataSpace memory_space(2, count);
        dataset.write(
            data, H5::PredType::NATIVE_DOUBLE, memory_space, file_space);
    }

    static void AppendScalar(H5::DataSet& dataset, hsize_t row,
                             double value) {
        const hsize_t new_size[1]{row + 1};
        dataset.extend(new_size);
        H5::DataSpace file_space = dataset.getSpace();
        const hsize_t offset[1]{row};
        const hsize_t count[1]{1};
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        H5::DataSpace memory_space(1, count);
        dataset.write(
            &value, H5::PredType::NATIVE_DOUBLE, memory_space, file_space);
    }

    static void AppendLength(H5::DataSet& dataset, hsize_t row,
                             uint64_t value) {
        const hsize_t new_size[1]{row + 1};
        dataset.extend(new_size);
        H5::DataSpace file_space = dataset.getSpace();
        const hsize_t offset[1]{row};
        const hsize_t count[1]{1};
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        H5::DataSpace memory_space(1, count);
        dataset.write(
            &value, H5::PredType::NATIVE_UINT64, memory_space, file_space);
    }

    static void AppendImage(H5::DataSet& image_dataset,
                            H5::DataSet& length_dataset, hsize_t row,
                            const std::vector<uint8_t>& jpeg) {
        std::vector<uint8_t> padded(kMaxJpegBytes, 0);
        std::copy(jpeg.begin(), jpeg.end(), padded.begin());
        const hsize_t new_size[2]{row + 1, kMaxJpegBytes};
        image_dataset.extend(new_size);
        H5::DataSpace file_space = image_dataset.getSpace();
        const hsize_t offset[2]{row, 0};
        const hsize_t count[2]{1, kMaxJpegBytes};
        file_space.selectHyperslab(H5S_SELECT_SET, count, offset);
        H5::DataSpace memory_space(2, count);
        image_dataset.write(
            padded.data(), H5::PredType::NATIVE_UINT8, memory_space,
            file_space);
        AppendLength(
            length_dataset, row, static_cast<uint64_t>(jpeg.size()));
    }

    void CloseHandles() {
        qpos_ = H5::DataSet();
        actions_ = H5::DataSet();
        control_mode_ = H5::DataSet();
        gate_ = H5::DataSet();
        image_timestamp_ = H5::DataSet();
        state_timestamp_ = H5::DataSet();
        state_time_offset_ = H5::DataSet();
        camera_sync_skew_ = H5::DataSet();
        image_timestamp_left_ = H5::DataSet();
        image_timestamp_right_ = H5::DataSet();
        image_timestamp_head_ = H5::DataSet();
        image_left_ = H5::DataSet();
        image_right_ = H5::DataSet();
        image_head_ = H5::DataSet();
        image_left_len_ = H5::DataSet();
        image_right_len_ = H5::DataSet();
        image_head_len_ = H5::DataSet();
        file_.reset();
    }

    int episode_index_{0};
    size_t frame_count_{0};
    hsize_t state_dimension_{0};
    double max_camera_sync_skew_ms_{0.0};
    double max_abs_state_image_offset_ms_{0.0};
    std::filesystem::path temporary_path_;
    std::filesystem::path final_path_;
    std::unique_ptr<H5::H5File> file_;
    H5::DataSet qpos_;
    H5::DataSet actions_;
    H5::DataSet control_mode_;
    H5::DataSet gate_;
    H5::DataSet image_timestamp_;
    H5::DataSet state_timestamp_;
    H5::DataSet state_time_offset_;
    H5::DataSet camera_sync_skew_;
    H5::DataSet image_timestamp_left_;
    H5::DataSet image_timestamp_right_;
    H5::DataSet image_timestamp_head_;
    H5::DataSet image_left_;
    H5::DataSet image_right_;
    H5::DataSet image_head_;
    H5::DataSet image_left_len_;
    H5::DataSet image_right_len_;
    H5::DataSet image_head_len_;
};

template <>
const H5::PredType& EpisodeWriter::NativeType<int32_t>() {
    return H5::PredType::NATIVE_INT32;
}

template <>
const H5::PredType& EpisodeWriter::NativeType<uint8_t>() {
    return H5::PredType::NATIVE_UINT8;
}

template <>
const H5::PredType& EpisodeWriter::NativeType<uint64_t>() {
    return H5::PredType::NATIVE_UINT64;
}

template <>
const H5::PredType& EpisodeWriter::NativeType<double>() {
    return H5::PredType::NATIVE_DOUBLE;
}

}  // namespace

class RecordingManager::Impl {
public:
    friend class RecordingManager;

    explicit Impl(RecordingConfig config)
        : config_(std::move(config)),
          status_(config_.enabled ? Status::Unavailable : Status::Disabled) {}

    ~Impl() {
        Shutdown();
    }

    bool Start() {
        if (!config_.enabled) {
            return true;
        }
        if (started_.exchange(true)) {
            return true;
        }
        try {
            std::filesystem::create_directories(config_.dataset_root);
            config_.dataset_root =
                std::filesystem::absolute(config_.dataset_root).lexically_normal();
            CleanupStaleTemporaryFiles(config_.dataset_root);
            episode_index_ = DetermineNextEpisodeIndex(config_.dataset_root);
        } catch (const std::exception& error) {
            started_.store(false);
            status_.store(Status::Unavailable);
            std::cerr << "[Recording] dataset initialization failed: "
                      << error.what() << std::endl;
            return false;
        }

        writer_running_.store(true);
        writer_thread_ = std::thread(&Impl::WriterLoop, this);
#ifdef DR02_PRO_HAS_REALSENSE
        camera_running_.store(true);
        camera_thread_ = std::thread(&Impl::CameraLoop, this);
#else
        status_.store(Status::Unavailable);
        std::cerr
            << "[Recording] RealSense support is not present in this build; "
               "teleoperation remains available but recording is disabled."
            << std::endl;
#endif
        std::cout << "[Recording] dataset root: " << config_.dataset_root
                  << ", next episode=" << episode_index_ << std::endl;
        return true;
    }

    void Shutdown() {
        if (!started_.exchange(false)) {
            return;
        }
        RequestAbortEpisode("state_machine shutdown");
        accepting_samples_.store(false);

        camera_running_.store(false);
        if (camera_thread_.joinable()) {
            camera_thread_.join();
        }

        writer_running_.store(false);
        operation_cv_.notify_all();
        if (writer_thread_.joinable()) {
            writer_thread_.join();
        }
        writer_.Abort();
        status_.store(Status::Disabled);
        camera_ready_.store(false);
    }

    bool RequestStartEpisode() {
        Status expected = Status::Idle;
        if (!status_.compare_exchange_strong(expected, Status::Starting)) {
            std::cout << "[Recording] start ignored: recorder is "
                      << StatusName(expected) << std::endl;
            return false;
        }
        if (!camera_ready_.load()) {
            status_.store(Status::Unavailable);
            std::cout << "[Recording] start rejected: all three cameras are required"
                      << std::endl;
            return false;
        }
        abort_requested_.store(false);
        EnqueueControl(OperationKind::Start);
        return true;
    }

    bool RequestFinishEpisode() {
        Status expected = Status::Recording;
        if (!status_.compare_exchange_strong(expected, Status::Finalizing)) {
            std::cout << "[Recording] finish ignored: recorder is "
                      << StatusName(expected) << std::endl;
            return false;
        }
        accepting_samples_.store(false);
        EnqueueControl(OperationKind::Finish);
        return true;
    }

    bool RequestAbortEpisode(const std::string& reason) {
        Status current = status_.load();
        while (IsActiveStatus(current)) {
            if (status_.compare_exchange_weak(current, Status::Aborting)) {
                abort_requested_.store(true);
                accepting_samples_.store(false);
                {
                    std::lock_guard<std::mutex> lock(operation_mutex_);
                    operation_queue_.erase(
                        std::remove_if(
                            operation_queue_.begin(), operation_queue_.end(),
                            [](const Operation& operation) {
                                return operation.kind == OperationKind::Sample;
                            }),
                        operation_queue_.end());
                    operation_queue_.push_back(
                        Operation{OperationKind::Abort, nullptr, reason});
                }
                operation_cv_.notify_one();
                std::cout << "[Recording] abort requested: " << reason
                          << std::endl;
                return true;
            }
        }
        return false;
    }

    bool RequestDeleteLastEpisode() {
        Status current = status_.load();
        if (current != Status::Idle) {
            std::cout << "[Recording] delete ignored: recorder is "
                      << StatusName(current) << std::endl;
            return false;
        }
        if (!status_.compare_exchange_strong(current, Status::Deleting)) {
            return false;
        }
        EnqueueControl(OperationKind::Delete);
        return true;
    }

    void AddStateFrame(const RecordingStateFrame& frame) {
        if (!accepting_samples_.load() ||
            status_.load() != Status::Recording) {
            return;
        }
        const size_t expected_dimension = config_.schema.Dimension();
        if (frame.qpos.size() != expected_dimension ||
            frame.action.size() != expected_dimension) {
            RequestAbortEpisode("state dimension mismatch");
            return;
        }
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_frames_.push_back(frame);
        while (state_frames_.size() > kMaxStateFrames) {
            state_frames_.pop_front();
        }
    }

    Status GetStatus() const {
        return status_.load();
    }

    static const char* StatusName(Status status) {
        switch (status) {
            case Status::Disabled:
                return "disabled";
            case Status::Unavailable:
                return "unavailable";
            case Status::Idle:
                return "idle";
            case Status::Starting:
                return "starting";
            case Status::Recording:
                return "recording";
            case Status::Finalizing:
                return "finalizing";
            case Status::Aborting:
                return "aborting";
            case Status::Deleting:
                return "deleting";
        }
        return "unknown";
    }

private:
    struct CameraFrame {
        double timestamp_ms{0.0};
        std::vector<uint8_t> rgb;
    };

    struct SyncedSample {
        double image_timestamp_ms{0.0};
        std::array<double, 3> image_timestamps_ms{0.0, 0.0, 0.0};
        double camera_sync_skew_ms{0.0};
        RecordingStateFrame state;
        std::array<std::vector<uint8_t>, 3> images;
    };

    enum class OperationKind {
        Start,
        Sample,
        Finish,
        Abort,
        Delete,
    };

    struct Operation {
        OperationKind kind{OperationKind::Abort};
        std::shared_ptr<SyncedSample> sample;
        std::string reason;
    };

    void EnqueueControl(OperationKind kind) {
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            operation_queue_.push_back(Operation{kind, nullptr, {}});
        }
        operation_cv_.notify_one();
    }

    bool EnqueueSample(const std::shared_ptr<SyncedSample>& sample) {
        bool overflow = false;
        {
            std::lock_guard<std::mutex> lock(operation_mutex_);
            if (!accepting_samples_.load() ||
                status_.load() != Status::Recording) {
                return false;
            }
            const size_t samples = static_cast<size_t>(std::count_if(
                operation_queue_.begin(), operation_queue_.end(),
                [](const Operation& operation) {
                    return operation.kind == OperationKind::Sample;
                }));
            if (samples >= kMaxPendingSamples) {
                overflow = true;
            } else {
                operation_queue_.push_back(
                    Operation{OperationKind::Sample, sample, {}});
            }
        }
        if (overflow) {
            RequestAbortEpisode("HDF5 writer queue overflow");
            return false;
        }
        operation_cv_.notify_one();
        return true;
    }

    void WriterLoop() {
        while (true) {
            Operation operation;
            {
                std::unique_lock<std::mutex> lock(operation_mutex_);
                operation_cv_.wait(lock, [this]() {
                    return !operation_queue_.empty() ||
                           !writer_running_.load();
                });
                if (operation_queue_.empty()) {
                    if (!writer_running_.load()) {
                        return;
                    }
                    continue;
                }
                operation = std::move(operation_queue_.front());
                operation_queue_.pop_front();
            }

            try {
                ProcessOperation(operation);
            } catch (const std::exception& error) {
                std::cerr << "[Recording] writer failure: " << error.what()
                          << std::endl;
                abort_requested_.store(true);
                accepting_samples_.store(false);
                writer_.Abort();
                ClearEpisodeBuffers();
                SetInactiveStatus();
            }
        }
    }

    void ProcessOperation(const Operation& operation) {
        switch (operation.kind) {
            case OperationKind::Start: {
                if (abort_requested_.load() || !camera_ready_.load()) {
                    SetInactiveStatus();
                    return;
                }
                ClearEpisodeBuffers();
                writer_.Open(config_.dataset_root, episode_index_,
                             config_.schema);
                if (abort_requested_.load()) {
                    writer_.Abort();
                    SetInactiveStatus();
                    return;
                }
                status_.store(Status::Recording);
                accepting_samples_.store(true);
                std::cout << "[Recording] episode " << episode_index_
                          << " started" << std::endl;
                break;
            }
            case OperationKind::Sample:
                if (operation.sample && writer_.IsOpen() &&
                    !abort_requested_.load()) {
                    writer_.Append(
                        operation.sample->state,
                        operation.sample->image_timestamp_ms,
                        operation.sample->image_timestamps_ms,
                        operation.sample->camera_sync_skew_ms,
                        operation.sample->images);
                }
                break;
            case OperationKind::Finish: {
                accepting_samples_.store(false);
                if (abort_requested_.load()) {
                    writer_.Abort();
                } else if (writer_.Finish()) {
                    ++episode_index_;
                } else {
                    std::cout << "[Recording] empty episode discarded"
                              << std::endl;
                }
                ClearEpisodeBuffers();
                abort_requested_.store(false);
                SetInactiveStatus();
                break;
            }
            case OperationKind::Abort:
                accepting_samples_.store(false);
                writer_.Abort();
                ClearEpisodeBuffers();
                abort_requested_.store(false);
                SetInactiveStatus();
                break;
            case OperationKind::Delete:
                DeleteLastEpisodeNow();
                SetInactiveStatus();
                break;
        }
    }

    void SetInactiveStatus() {
        status_.store(
            camera_ready_.load() ? Status::Idle : Status::Unavailable);
    }

    void ClearEpisodeBuffers() {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_frames_.clear();
        }
        {
            std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
            for (auto& buffer : camera_frames_) {
                buffer.clear();
            }
        }
    }

    void DeleteLastEpisodeNow() {
        const int next = DetermineNextEpisodeIndex(config_.dataset_root);
        const int target_index = next - 1;
        if (target_index < 0) {
            std::cout << "[Recording] no episode to delete" << std::endl;
            episode_index_ = 0;
            return;
        }
        const auto target =
            config_.dataset_root /
            ("episode_" + std::to_string(target_index) + ".hdf5");
        std::error_code error;
        if (std::filesystem::remove(target, error)) {
            episode_index_ = DetermineNextEpisodeIndex(config_.dataset_root);
            std::cout << "[Recording] deleted: " << target << std::endl;
        } else {
            std::cerr << "[Recording] delete failed for " << target << ": "
                      << (error ? error.message() : "file not found")
                      << std::endl;
            episode_index_ = DetermineNextEpisodeIndex(config_.dataset_root);
        }
    }

    bool FindNearestState(double timestamp_ms,
                          RecordingStateFrame& matched) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_frames_.empty()) {
            return false;
        }
        size_t best_index = 0;
        double best_difference = std::numeric_limits<double>::max();
        for (size_t i = 0; i < state_frames_.size(); ++i) {
            const double difference =
                std::abs(state_frames_[i].timestamp_ms - timestamp_ms);
            if (difference < best_difference) {
                best_difference = difference;
                best_index = i;
            }
        }
        matched = state_frames_[best_index];
        state_frames_.erase(
            state_frames_.begin(), state_frames_.begin() + best_index + 1);
        return true;
    }

    void TrySynchronizeOne() {
        std::array<std::vector<uint8_t>, 3> images;
        std::array<double, 3> image_timestamps_ms{0.0, 0.0, 0.0};
        double minimum_timestamp = 0.0;
        double camera_sync_skew_ms = 0.0;
        {
            std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
            if (std::any_of(
                    camera_frames_.begin(), camera_frames_.end(),
                    [](const auto& frames) { return frames.empty(); })) {
                return;
            }

            double maximum_timestamp =
                camera_frames_[0].front().timestamp_ms;
            minimum_timestamp = maximum_timestamp;
            size_t earliest_index = 0;
            for (size_t i = 1; i < camera_frames_.size(); ++i) {
                const double timestamp =
                    camera_frames_[i].front().timestamp_ms;
                if (timestamp < minimum_timestamp) {
                    minimum_timestamp = timestamp;
                    earliest_index = i;
                }
                maximum_timestamp = std::max(maximum_timestamp, timestamp);
            }
            if (maximum_timestamp - minimum_timestamp > kSyncToleranceMs) {
                camera_frames_[earliest_index].pop_front();
                return;
            }
            camera_sync_skew_ms = maximum_timestamp - minimum_timestamp;
        }

        RecordingStateFrame matched_state;
        if (!FindNearestState(minimum_timestamp, matched_state)) {
            std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
            size_t earliest_index = 0;
            for (size_t i = 1; i < camera_frames_.size(); ++i) {
                if (camera_frames_[i].front().timestamp_ms <
                    camera_frames_[earliest_index].front().timestamp_ms) {
                    earliest_index = i;
                }
            }
            camera_frames_[earliest_index].pop_front();
            return;
        }

        {
            std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
            if (std::any_of(
                    camera_frames_.begin(), camera_frames_.end(),
                    [](const auto& frames) { return frames.empty(); })) {
                return;
            }
            for (size_t i = 0; i < camera_frames_.size(); ++i) {
                image_timestamps_ms[i] =
                    camera_frames_[i].front().timestamp_ms;
                images[i] = std::move(camera_frames_[i].front().rgb);
                camera_frames_[i].pop_front();
            }
        }

        auto sample = std::make_shared<SyncedSample>();
        sample->image_timestamp_ms = minimum_timestamp;
        sample->image_timestamps_ms = image_timestamps_ms;
        sample->camera_sync_skew_ms = camera_sync_skew_ms;
        sample->state = matched_state;
        sample->images = std::move(images);
        EnqueueSample(sample);
    }

    void TrimIdleCameraBuffers() {
        std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
        for (auto& frames : camera_frames_) {
            while (frames.size() > 1) {
                frames.pop_front();
            }
        }
    }

#ifdef DR02_PRO_RECORDING_TEST_API
    void SetSyntheticCamerasReadyForTesting() {
        camera_ready_.store(true);
        if (status_.load() == Status::Unavailable) {
            status_.store(Status::Idle);
        }
    }

    void AddCameraFrameForTesting(
        size_t camera_index, double timestamp_ms,
        const std::vector<uint8_t>& rgb) {
        if (camera_index >= camera_frames_.size()) {
            throw std::out_of_range("invalid synthetic camera index");
        }
        {
            std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
            camera_frames_[camera_index].push_back(
                CameraFrame{timestamp_ms, rgb});
        }
        if (status_.load() == Status::Recording) {
            TrySynchronizeOne();
        }
    }
#endif

#ifdef DR02_PRO_HAS_REALSENSE
    void CameraLoop() {
        camera_last_retry_ms_.fill(0.0);
        camera_last_ok_ms_.fill(0.0);
        camera_online_.fill(false);

        while (camera_running_.load()) {
            const double now_ms = WallNowMs();
            for (size_t i = 0; i < camera_pipelines_.size(); ++i) {
                if (!camera_online_[i] &&
                    (camera_last_retry_ms_[i] == 0.0 ||
                     now_ms - camera_last_retry_ms_[i] >= kCameraRetryMs)) {
                    TryStartCamera(i, now_ms);
                }
            }

            for (size_t i = 0; i < camera_pipelines_.size(); ++i) {
                if (!camera_online_[i]) {
                    continue;
                }
                PollCamera(i);
                if (camera_last_ok_ms_[i] > 0.0 &&
                    WallNowMs() - camera_last_ok_ms_[i] >= kCameraStallMs) {
                    MarkCameraOffline(i, "no valid frame for 2 seconds");
                }
            }

            UpdateCameraAvailability();
            if (status_.load() == Status::Recording &&
                camera_ready_.load()) {
                TrySynchronizeOne();
            } else {
                TrimIdleCameraBuffers();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        for (size_t i = 0; i < camera_pipelines_.size(); ++i) {
            StopCamera(i);
        }
        camera_ready_.store(false);
    }

    void TryStartCamera(size_t index, double now_ms) {
        camera_last_retry_ms_[index] = now_ms;
        try {
            auto pipeline = std::make_unique<rs2::pipeline>();
            rs2::config config;
            config.enable_device(kCameraSerials[index]);
            config.enable_stream(
                RS2_STREAM_COLOR, kWidth, kHeight, RS2_FORMAT_RGB8, kFps);
            const rs2::pipeline_profile profile = pipeline->start(config);
            for (auto&& sensor : profile.get_device().query_sensors()) {
                if (sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) {
                    sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.0f);
                }
                if (sensor.supports(RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
                    sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
                }
            }
            camera_pipelines_[index] = std::move(pipeline);
            camera_online_[index] = true;
            camera_last_ok_ms_[index] = now_ms;
            {
                std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
                camera_frames_[index].clear();
            }
            std::cout << "[Recording] camera " << kCameraNames[index]
                      << " online (" << kCameraSerials[index] << ")"
                      << std::endl;
        } catch (const std::exception& error) {
            camera_online_[index] = false;
            camera_pipelines_[index].reset();
            std::cerr << "[Recording] camera " << kCameraNames[index]
                      << " unavailable: " << error.what() << std::endl;
        }
    }

    void PollCamera(size_t index) {
        try {
            rs2::frameset frames;
            if (!camera_pipelines_[index]->poll_for_frames(&frames)) {
                return;
            }
            const rs2::video_frame color = frames.get_color_frame();
            if (!color || !color.get_data() || color.get_width() != kWidth ||
                color.get_height() != kHeight) {
                return;
            }
            CameraFrame frame;
            frame.timestamp_ms = WallNowMs();
            frame.rgb.resize(
                static_cast<size_t>(kWidth * kHeight * kChannels));
            std::memcpy(
                frame.rgb.data(), color.get_data(), frame.rgb.size());
            camera_last_ok_ms_[index] = frame.timestamp_ms;
            {
                std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
                camera_frames_[index].push_back(std::move(frame));
                while (camera_frames_[index].size() > kMaxCameraFrames) {
                    camera_frames_[index].pop_front();
                }
            }
        } catch (const std::exception& error) {
            MarkCameraOffline(index, error.what());
        }
    }

    void MarkCameraOffline(size_t index, const std::string& reason) {
        if (!camera_online_[index]) {
            return;
        }
        std::cerr << "[Recording] camera " << kCameraNames[index]
                  << " offline: " << reason << std::endl;
        StopCamera(index);
        camera_last_retry_ms_[index] = WallNowMs();
        UpdateCameraAvailability();
    }

    void StopCamera(size_t index) {
        if (camera_pipelines_[index]) {
            try {
                camera_pipelines_[index]->stop();
            } catch (...) {
            }
        }
        camera_pipelines_[index].reset();
        camera_online_[index] = false;
        camera_last_ok_ms_[index] = 0.0;
        std::lock_guard<std::mutex> lock(camera_buffer_mutex_);
        camera_frames_[index].clear();
    }

    void UpdateCameraAvailability() {
        const bool all_online =
            std::all_of(camera_online_.begin(), camera_online_.end(),
                        [](bool online) { return online; });
        const bool was_ready = camera_ready_.exchange(all_online);
        const Status current = status_.load();
        if (all_online && !was_ready && current == Status::Unavailable) {
            status_.store(Status::Idle);
            std::cout << "[Recording] all three cameras ready" << std::endl;
        } else if (!all_online && was_ready) {
            if (current == Status::Idle) {
                status_.store(Status::Unavailable);
            } else if (IsActiveStatus(current)) {
                RequestAbortEpisode("camera stream lost");
            }
        }
    }

#endif

    RecordingConfig config_;
    std::atomic<Status> status_{Status::Disabled};
    std::atomic<bool> started_{false};
    std::atomic<bool> camera_ready_{false};
    std::atomic<bool> accepting_samples_{false};
    std::atomic<bool> abort_requested_{false};
    std::atomic<bool> camera_running_{false};
    std::atomic<bool> writer_running_{false};

    int episode_index_{0};
    EpisodeWriter writer_;

    std::mutex state_mutex_;
    std::deque<RecordingStateFrame> state_frames_;
    std::mutex camera_buffer_mutex_;
    std::array<std::deque<CameraFrame>, 3> camera_frames_;

    std::mutex operation_mutex_;
    std::condition_variable operation_cv_;
    std::deque<Operation> operation_queue_;
    std::thread writer_thread_;
    std::thread camera_thread_;

#ifdef DR02_PRO_HAS_REALSENSE
    std::array<std::unique_ptr<rs2::pipeline>, 3> camera_pipelines_;
    std::array<bool, 3> camera_online_{false, false, false};
    std::array<double, 3> camera_last_ok_ms_{0.0, 0.0, 0.0};
    std::array<double, 3> camera_last_retry_ms_{0.0, 0.0, 0.0};
#endif
};

RecordingManager::RecordingManager(RecordingConfig config)
    : impl_(std::make_unique<Impl>(std::move(config))) {}

RecordingManager::~RecordingManager() = default;

bool RecordingManager::Start() {
    return impl_->Start();
}

void RecordingManager::Shutdown() {
    impl_->Shutdown();
}

bool RecordingManager::RequestStartEpisode() {
    return impl_->RequestStartEpisode();
}

bool RecordingManager::RequestFinishEpisode() {
    return impl_->RequestFinishEpisode();
}

bool RecordingManager::RequestAbortEpisode(const std::string& reason) {
    return impl_->RequestAbortEpisode(reason);
}

bool RecordingManager::RequestDeleteLastEpisode() {
    return impl_->RequestDeleteLastEpisode();
}

void RecordingManager::AddStateFrame(const RecordingStateFrame& frame) {
    impl_->AddStateFrame(frame);
}

RecordingManager::Status RecordingManager::GetStatus() const {
    return impl_->GetStatus();
}

bool RecordingManager::IsEnabled() const {
    return GetStatus() != Status::Disabled;
}

bool RecordingManager::IsRecording() const {
    return GetStatus() == Status::Recording;
}

#ifdef DR02_PRO_RECORDING_TEST_API
void RecordingManager::SetSyntheticCamerasReadyForTesting() {
    impl_->SetSyntheticCamerasReadyForTesting();
}

void RecordingManager::AddCameraFrameForTesting(
    size_t camera_index, double timestamp_ms,
    const std::vector<uint8_t>& rgb) {
    impl_->AddCameraFrameForTesting(camera_index, timestamp_ms, rgb);
}
#endif

}  // namespace deep_robotics::dr02_pro
