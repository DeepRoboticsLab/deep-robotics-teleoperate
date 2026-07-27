# Changelog

## [Unreleased]

### Added

- Pico upper-body teleoperation with headset-driven waist control and dual-arm
  6D inverse kinematics.
- Integration with a pinned `deep_robotics_model` Git submodule using
  `DR02/urdf/pro/DR02-pro.urdf`.
- Pico stream timeout protection, joint-target filtering, software limits, and
  elbow-to-torso soft avoidance.

### Changed

- Simplified the state machine to `Emergency`, `ZeroPos`, `Teleoperation`, and
  `Down`.
- Renamed the former start-pose state to `Down`; the keyboard `d` command
  enters that state.
- Redefined the former idle pose-hold state as `Emergency`; the keyboard `e`
  command enters that state.
- Updated the IK reduction for the full 31-DoF DR02 Pro URDF by locking the
  waist, neck, and both legs, leaving the 14 arm DoFs required by IK.
- Removed the obsolete z-axis wrist-rotation inversion for the current DR02
  Pro URDF.
- Reworked the English and Chinese operating guides with separate, complete
  simulation and real-robot procedures.
- Aligned the 500 Hz safety-monitor period and Eigen-safe structure layout
  with the common SDK.

### Removed

- RL control, ONNX policy inference, and their ONNX Runtime dependency.
- The joint-damping state and its control entry points.
- Low-level, high-level, peripheral, and audio examples.
- Bundled robot model files and the unused bundled gamepad library.

## [0.0.1] - 2026-06-26

### Added
- Initial DR02 Pro ROS2 SDK package.
- Added state machine, low-level, high-level, peripheral, and audio examples.
