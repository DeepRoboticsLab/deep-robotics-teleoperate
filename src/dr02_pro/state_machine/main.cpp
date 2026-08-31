/**
 * @file main.cpp
 * @brief State-machine executable entry point.
 * @author DEEPRobotics Teleoperate
 * @date 2026-06-27
 *
 * @copyright Copyright (c) 2026 DEEPRobotics Teleoperate
 *
 */
#include <iostream>

#include "humanoid_state_machine.hpp"
#include "runtime_options.hpp"

using namespace deep_robotics::dr02_pro;

int main(int argc, char** argv) {
  RuntimeOptions options;
  if (!ParseRuntimeOptions(argc, argv, options, std::cerr)) {
    PrintUsage(argv[0], std::cerr);
    return 2;
  }
  if (options.help_requested) {
    PrintUsage(argv[0], std::cout);
    return 0;
  }

  std::cout << "State Machine Start Running" << std::endl;
  rclcpp::init(argc, argv);

  std::shared_ptr<HumanoidStateMachine> fsm =
      std::make_shared<HumanoidStateMachine>(RobotName::DR2PRO,
                                             options.recording,
                                             options.end_effector,
                                             options.periodic_log_enabled);
  fsm->Start();
  fsm->Run();
  fsm->Stop();

  rclcpp::shutdown();
  return 0;
}
