# DEEPRobotics Teleoperate

在 [YouTube](https://youtu.be/3T9cKVp1rlc?si=tNFjaenysMMZ8Y-F) 或 [哔哩哔哩](https://b23.tv/GHEIDgg) 观看遥操作演示视频。

[![ROS 2](https://img.shields.io/badge/ROS-2-22314E?logo=ros)](https://www.ros.org/)
[![PICO XR](https://img.shields.io/badge/XR-PICO-00AEEF)](https://www.picoxr.com/cn/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://zh.cppreference.com/w/cpp/17)
[![x86_64](https://img.shields.io/badge/platform-x86__64-lightgrey)](src/dr02_pro/docs/SIMULATION_CN.md)
[![ARM64](https://img.shields.io/badge/platform-ARM64-lightgrey)](src/dr02_pro/docs/REAL_ROBOT_CN.md)
[![License](https://img.shields.io/badge/license-BSD%203--Clause-blue)](LICENSE)
[![Discord](https://img.shields.io/badge/Discord-5865F2?logo=discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

[English](README.md) | **简体中文**

DEEPRobotics Teleoperate 是面向 DR02 Pro 人形机器人的 ROS 2 遥操作项目。项目将 PICO
头显与手柄输入转换为腰部和双臂指令，可用于 DR02 Pro 实机或 MuJoCo 仿真。受支持的末端
执行器通过运行参数按需启用；真机数据采集可记录纯双臂或双臂加末端执行器数据。

## 系统概览

```text
PICO + XRoboToolkit ──► Robotics Service / PXREA ──► state_machine
                                                        │
                    /JOINTS_DATA ◄── DR02 Pro / MuJoCo ─┤─► /JOINTS_CMD
              可选末端执行器反馈 ◄─────────────────────└─► 可选末端执行器指令
```

## 获取源码

克隆时同时初始化 submodule，以获取经过验证的 DR02 Pro 模型：

```bash
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

已有仓库可执行：

```bash
git submodule update --init --recursive
```

## 从这里开始

请先阅读 [DR02 Pro Pico 遥操作指南](src/dr02_pro/README_CN.md)，再选择仿真或真机流程。
仿真器和 ROS 2 消息接口由独立仓库提供。控制实机时不得使用仿真编译配置。

## 相关仓库

- [deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)：
  ROS 2 消息接口库，软件包名为 `drdds`。
- [deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model)：
  遥操作 IK 使用的 DR02 Pro URDF 和网格模型；本仓库通过 submodule 固定其已验证版本。
- [deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation)：
  MuJoCo 仿真环境；仿真器不包含在本仓库中。

> [!WARNING]
>
> 本软件会直接发布运动指令。`Emergency` 是软件姿态保持，不是硬件急停。首次使用必须先在
> 仿真中验证；控制实机时必须确保物理急停随时可用。
