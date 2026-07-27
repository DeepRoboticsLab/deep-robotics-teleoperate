# DEEPRobotics Teleoperate

[![ROS 2](https://img.shields.io/badge/ROS-2-22314E?logo=ros)](https://www.ros.org/)
[![PICO XR](https://img.shields.io/badge/XR-PICO-00AEEF)](https://www.picoxr.com/cn/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://cppreference.cn/w/cpp/17)
[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)
[![x86_64](https://img.shields.io/badge/platform-x86__64-lightgrey)](src/dr02_pro/docs/SIMULATION.md)
[![ARM64](https://img.shields.io/badge/platform-ARM64-lightgrey)](src/dr02_pro/docs/REAL_ROBOT.md)
[![License](https://img.shields.io/badge/license-BSD%203--Clause-blue)](https://opensource.org/license/bsd-3-clause)

**English** | [简体中文](README_CN.md)

DEEPRobotics Teleoperate provides the ROS 2 package required for DR02 Pro Pico
upper-body teleoperation. It receives headset and controller data through
XRoboToolkit, calculates waist and dual-arm targets, and publishes complete
31-DoF joint commands to a real DR02 Pro robot or a MuJoCo simulation.

## System Overview

```text
Pico XRoboToolkit
        │ headset and controller data
        ▼
Robotics Service ── PXREA ──► dr02_tele/state_machine
                                      ▲            │
                         /JOINTS_DATA │            │ /JOINTS_CMD
                                      │            ▼
                              DR02 Pro robot or MuJoCo simulator
```

## Repository Layout

```text
src/common/          Shared C++ types and utilities
src/dr02_pro/        DR02 Pro teleoperation package and documentation
third_party/         Bundled x86/ARM XR, IK, and support dependencies
deep_robotics_model/ Pinned DR02 Pro models (Git submodule)
```

## Clone the Repository

Initialize the model submodule when cloning this repository:

```bash
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

If the repository was already obtained with a regular `git clone`, run the
following command from its root:

```bash
git submodule update --init --recursive
```

> [!NOTE]
>
> GitHub **Download ZIP** does not initialize submodules. Use the Git clone
> workflow above when preparing a complete runtime environment.

## Start Here

Read the [DR02 Pro Pico Teleoperation Guide](src/dr02_pro/README.md), then
choose the real-robot or simulation workflow for your target. Never use the
simulation build configuration to control a real robot.

## Related Repositories

- [deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg):
  ROS 2 message interface library. Its package name is `drdds`.
- [deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model):
  DR02 Pro URDF and mesh assets used by teleoperation IK. This repository pins
  its verified version as a submodule.
- [deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation):
  MuJoCo simulation environment. The simulator is not included in this
  repository.

> [!WARNING]
>
> Real-robot teleoperation directly publishes motion commands. Read the
> real-robot safety requirements before connecting this package to a robot.
