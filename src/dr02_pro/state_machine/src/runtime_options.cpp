#include "runtime_options.hpp"

#include <filesystem>
#include <ostream>

#include "end_effector_factory.hpp"

namespace deep_robotics::dr02_pro {

void PrintUsage(const char* program, std::ostream& output) {
    output << "Usage: " << program
           << " [--log] [--record] [--record-dir PATH] "
              "[--ee zhixing] [--ros-args ...]\n"
           << "  --log              Print Pico diagnostics once per second\n"
           << "  --record           Enable three-camera HDF5 recording\n"
           << "  --record-dir PATH  Dataset directory (requires --record)\n"
           << "  --ee NAME          Enable an end effector (supported: zhixing)\n";
}

bool ParseRuntimeOptions(int argc, char** argv, RuntimeOptions& options,
                         std::ostream& error) {
    options = RuntimeOptions{};
    bool log_set = false;
    bool record_set = false;
    bool record_dir_set = false;
    bool end_effector_set = false;

    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--ros-args") {
            break;
        }
        if (argument == "--help" || argument == "-h") {
            options.help_requested = true;
            return true;
        }
        if (argument == "--log") {
            if (log_set) {
                error << "Duplicate argument: --log\n";
                return false;
            }
            log_set = true;
            options.periodic_log_enabled = true;
            continue;
        }
        if (argument == "--record") {
            if (record_set) {
                error << "Duplicate argument: --record\n";
                return false;
            }
            record_set = true;
            options.recording.enabled = true;
            continue;
        }
        if (argument == "--record-dir") {
            if (record_dir_set) {
                error << "Duplicate argument: --record-dir\n";
                return false;
            }
            if (index + 1 >= argc ||
                std::string(argv[index + 1]).rfind("--", 0) == 0) {
                error << "--record-dir requires a path\n";
                return false;
            }
            record_dir_set = true;
            options.recording.dataset_root = argv[++index];
            continue;
        }
        if (argument == "--ee") {
            if (end_effector_set) {
                error << "Duplicate argument: --ee\n";
                return false;
            }
            if (index + 1 >= argc ||
                std::string(argv[index + 1]).rfind("--", 0) == 0) {
                error << "--ee requires a name\n";
                return false;
            }
            end_effector_set = true;
            options.end_effector = argv[++index];
            if (!IsSupportedEndEffector(options.end_effector) ||
                options.end_effector.empty()) {
                error << "Unsupported end effector: "
                      << options.end_effector << "\n";
                return false;
            }
            continue;
        }
        error << "Unknown argument: " << argument << "\n";
        return false;
    }

    if (record_dir_set && !options.recording.enabled) {
        error << "--record-dir requires --record\n";
        return false;
    }
    if (options.recording.enabled && !record_dir_set) {
        options.recording.dataset_root =
            std::filesystem::current_path() / "record_data";
    }
    if (options.recording.enabled) {
        options.recording.dataset_root = std::filesystem::absolute(
            options.recording.dataset_root).lexically_normal();
    }
    return true;
}

}  // namespace deep_robotics::dr02_pro
