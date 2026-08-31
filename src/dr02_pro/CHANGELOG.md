# Changelog

## [Unreleased]

### Added

- Pico upper-body teleoperation with headset-driven waist control and dual-arm
  6D inverse kinematics.
- Dual-gripper teleoperation that maps the left and right Pico grip values to
  `/gripper/position_cmd` and receives per-side feedback from
  `/gripper/position` through the new RK-side Zhixing SDK.
- Integration with a pinned `deep_robotics_model` Git submodule using
  `DR02/urdf/pro/DR02-pro.urdf`.
- Pico stream timeout protection, joint-target filtering, software limits, and
  elbow-to-torso soft avoidance.

### Changed

- Decoupled end-effector control from DR02 arm teleoperation. The default
  runtime creates no gripper topics; `--ee zhixing` enables the independently
  built Zhixing implementation from the sibling `src/end_effector` module.
- Made recording state width runtime-configurable: arms-only episodes use 14
  columns, while Zhixing episodes use the new arms-first 16-column layout and
  require fresh measured gripper feedback.
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
- Made Pico translation, rotation, and headset deadbands continuous so crossing
  a threshold cannot create a target step.
- Limited dual-arm IK to fresh Pico packets while keeping joint-space filtering
  on every robot-feedback update.
- Applied the Pinocchio `Jlog6` correction to the 6D pose-error Jacobian.
- Made IK accept only descending Cartesian-error steps, return the best
  numerical iterate, and hold the last issued arm command when a candidate is
  non-finite, inaccurate, worse than its seed, or requires a large joint jump.
- Aligned the controller translation scale (`1.2`), IK regularization
  (`0.002` joint posture and `0.02` seed continuity), and per-joint target
  advance (`0.5 rad`) with the CR1 teleoperation configuration.
- Published the critically damped smoother's arm velocity together with its
  position target.
- Changed the `0.5 rad` IK continuity threshold from whole-candidate rejection
  to a per-joint, per-solve raw-target advance limit so fast valid solutions
  continue through the smoother without resetting it.
- Changed joint-feedback timeout handling to enter `Emergency` after `100 ms`
  without periodic diagnostics, and removed unchanged joint values as a fault
  condition while complete feedback frames continue to arrive.
- Disabled the once-per-second Pico diagnostic by default and added the
  `--log` command-line option to enable it when needed.

### Removed

- Unused control parameters, robot/Pico/recording accessors, dormant command
  fields, obsolete math helpers, and write-only diagnostic state.
- The standalone manipulation build toggle; manipulation is now built only as
  the state machine's required implementation dependency.
- RL control, ONNX policy inference, and their ONNX Runtime dependency.
- The joint-damping state and its control entry points.
- Low-level, high-level, peripheral, and audio examples.
- Bundled robot model files and the unused bundled gamepad library.

## [0.0.1] - 2026-06-26

### Added
- Initial DR02 Pro ROS2 SDK package.
- Added state machine, low-level, high-level, peripheral, and audio examples.
