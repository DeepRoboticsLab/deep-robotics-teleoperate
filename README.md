# DEEPRobotics Teleoperate

Watch the teleoperation demo on [YouTube](https://youtu.be/3T9cKVp1rlc?si=tNFjaenysMMZ8Y-F) or [Bilibili](https://b23.tv/GHEIDgg).

[![ROS 2](https://img.shields.io/badge/ROS-2-22314E?logo=ros)](https://www.ros.org/)
[![PICO XR](https://img.shields.io/badge/XR-PICO-00AEEF)](https://www.picoxr.com/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://en.cppreference.com/w/cpp/17)
[![x86_64](https://img.shields.io/badge/platform-x86__64-lightgrey)](src/dr02_pro/docs/SIMULATION.md)
[![ARM64](https://img.shields.io/badge/platform-ARM64-lightgrey)](src/dr02_pro/docs/REAL_ROBOT.md)
[![License](https://img.shields.io/badge/license-BSD%203--Clause-blue)](LICENSE)
[![Discord](https://img.shields.io/badge/Discord-5865F2?logo=discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

**English** | [简体中文](README_CN.md)

DEEPRobotics Teleoperate is a ROS 2 teleoperation project for the DR02 Pro
humanoid robot. It converts PICO headset and controller input into waist and
dual-arm commands for either a real robot or a MuJoCo simulation. A runtime
option enables supported end effectors, and real-robot data collection can
record either arms only or arms plus the selected end effector.

## System Overview

```text
PICO + XRoboToolkit ──► Robotics Service / PXREA ──► state_machine
                                                        │
                    /JOINTS_DATA ◄── DR02 Pro / MuJoCo ─┤─► /JOINTS_CMD
        optional end-effector feedback ◄────────────────└─► optional end-effector command
```

## Get the Source

Clone with submodules to include the verified DR02 Pro model:

```bash
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

For an existing clone:

```bash
git submodule update --init --recursive
```

## Start Here

Read the [DR02 Pro Pico Teleoperation Guide](src/dr02_pro/README.md), then
choose the simulation or real-robot workflow. The simulator and ROS 2 message
interfaces are provided by separate repositories. Never use a simulation build
to control a real robot.

## Related Repositories

- [deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg):
  ROS 2 message interfaces provided through the `drdds` package.
- [deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model):
  the pinned DR02 Pro URDF and mesh assets used by teleoperation IK.
- [deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation):
  the separately maintained MuJoCo simulation environment.

> [!WARNING]
>
> This software publishes motion commands directly. `Emergency` is a software
> pose hold, not a hardware emergency stop. Validate in simulation first and
> keep a physical emergency stop immediately accessible on the real robot.
