#pragma once

#include <iosfwd>
#include <string>

#include "recording_manager.hpp"

namespace deep_robotics::dr02_pro {

struct RuntimeOptions {
    RecordingConfig recording;
    std::string end_effector;
    bool periodic_log_enabled{false};
    bool help_requested{false};
};

void PrintUsage(const char* program, std::ostream& output);
bool ParseRuntimeOptions(int argc, char** argv, RuntimeOptions& options,
                         std::ostream& error);

}  // namespace deep_robotics::dr02_pro
