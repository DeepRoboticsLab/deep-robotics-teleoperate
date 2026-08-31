#include "runtime_options.hpp"

#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using deep_robotics::dr02_pro::ParseRuntimeOptions;
using deep_robotics::dr02_pro::RuntimeOptions;

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

struct ParseResult {
    bool success{false};
    RuntimeOptions options;
    std::string error;
};

ParseResult Parse(std::vector<std::string> arguments) {
    std::vector<char*> argv;
    argv.reserve(arguments.size());
    for (std::string& argument : arguments) {
        argv.push_back(argument.data());
    }
    ParseResult result;
    std::ostringstream error;
    result.success = ParseRuntimeOptions(
        static_cast<int>(argv.size()), argv.data(), result.options, error);
    result.error = error.str();
    return result;
}

void RequireFailure(const std::vector<std::string>& arguments,
                    const std::string& expected_error) {
    const ParseResult result = Parse(arguments);
    Require(!result.success, "invalid arguments unexpectedly succeeded");
    Require(result.error.find(expected_error) != std::string::npos,
            "wrong parse error: " + result.error);
}

}  // namespace

int main() {
    try {
        const ParseResult defaults = Parse({"state_machine"});
        Require(defaults.success, "default arguments failed");
        Require(!defaults.options.recording.enabled,
                "recording enabled by default");
        Require(defaults.options.end_effector.empty(),
                "end effector enabled by default");

        const ParseResult recording =
            Parse({"state_machine", "--record"});
        Require(recording.success && recording.options.recording.enabled,
                "--record failed");
        Require(recording.options.recording.dataset_root ==
                    (std::filesystem::current_path() / "record_data"),
                "default recording path is wrong");

        const ParseResult end_effector =
            Parse({"state_machine", "--ee", "zhixing"});
        Require(end_effector.success &&
                    end_effector.options.end_effector == "zhixing",
                "--ee zhixing failed");

        const ParseResult combined = Parse(
            {"state_machine", "--ee", "zhixing", "--record-dir",
             "relative_data", "--record", "--log"});
        Require(combined.success, "combined options failed");
        Require(combined.options.recording.enabled &&
                    combined.options.periodic_log_enabled,
                "combined flags were not retained");
        Require(combined.options.recording.dataset_root ==
                    (std::filesystem::current_path() / "relative_data"),
                "recording path was not normalized");

        const ParseResult ros_arguments = Parse(
            {"state_machine", "--ee", "zhixing", "--ros-args", "-r",
             "__node:=teleop"});
        Require(ros_arguments.success &&
                    ros_arguments.options.end_effector == "zhixing",
                "ROS argument boundary was not honored");

        const ParseResult help = Parse({"state_machine", "--help"});
        Require(help.success && help.options.help_requested,
                "--help failed");

        RequireFailure({"state_machine", "--ee"},
                       "--ee requires a name");
        RequireFailure({"state_machine", "--ee", "unknown"},
                       "Unsupported end effector");
        RequireFailure(
            {"state_machine", "--ee", "zhixing", "--ee", "zhixing"},
            "Duplicate argument: --ee");
        RequireFailure({"state_machine", "--record", "--record"},
                       "Duplicate argument: --record");
        RequireFailure({"state_machine", "--record-dir", "data"},
                       "--record-dir requires --record");
        RequireFailure({"state_machine", "--recoed"},
                       "Unknown argument: --recoed");
    } catch (const std::exception& error) {
        std::cerr << "runtime_options_test failed: " << error.what()
                  << std::endl;
        return 1;
    }
    return 0;
}
