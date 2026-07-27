/**
 * @file pinocchio_urdf_compat.cpp
 * @brief ABI bridge for the legacy bundled x86 Pinocchio URDF parser.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#include <string>

#include "pinocchio/parsers/urdf.hpp"

namespace pinocchio::urdf::details {

// The x86 libpinocchio.a exports this pre-mimic overload, while the headers
// bundled with this SDK declare the three-argument version below.
void parseRootTree(const std::string& filename, UrdfVisitorBase& model);

void parseRootTree(const std::string& filename, UrdfVisitorBase& model, const bool mimic) {
    // Legacy x86 Pinocchio has no mimic-joint switch. The DR02 upper-body URDF
    // does not use mimic joints, so forwarding the regular tree parse is safe.
    (void)mimic;
    using LegacyParseRootTree = void (*)(const std::string&, UrdfVisitorBase&);
    const auto legacy_parse = static_cast<LegacyParseRootTree>(&parseRootTree);
    legacy_parse(filename, model);
}

}  // namespace pinocchio::urdf::details
