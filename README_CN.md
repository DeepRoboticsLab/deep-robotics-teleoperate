# DEEPRobotics Teleoperate

[![ROS 2](https://img.shields.io/badge/ROS-2-22314E?logo=ros)](https://www.ros.org/)
[![PICO XR](https://img.shields.io/badge/XR-PICO-00AEEF)](https://www.picoxr.com/cn/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=cplusplus)](https://cppreference.cn/w/cpp/17)
[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)
[![x86_64](https://img.shields.io/badge/platform-x86__64-lightgrey)](src/dr02_pro/docs/SIMULATION_CN.md)
[![ARM64](https://img.shields.io/badge/platform-ARM64-lightgrey)](src/dr02_pro/docs/REAL_ROBOT_CN.md)
[![License](https://img.shields.io/badge/license-BSD%203--Clause-blue)](https://opensource.org/license/bsd-3-clause)

[English](README.md) | **简体中文**

DEEPRobotics Teleoperate 提供 DR02 Pro Pico 上半身遥操所需的基于 ROS 2 开发的软件包。它通过
XRoboToolkit 接收头显和手柄数据，计算腰部与双臂目标，并向 DR02 Pro 实机或 MuJoCo
仿真发布完整的 31 自由度关节指令。

## 系统概览

```text
Pico XRoboToolkit
        │ 头显和手柄数据
        ▼
Robotics Service ── PXREA ──► dr02_tele/state_machine
                                      ▲            │
                         /JOINTS_DATA │            │ /JOINTS_CMD
                                      │            ▼
                               DR02 Pro 实机或 MuJoCo 仿真器
```

## 仓库结构

```text
src/common/          公共 C++ 类型与工具
src/dr02_pro/        DR02 Pro 遥操功能包及文档
third_party/         随仓库提供的 x86/ARM XR、IK 和辅助依赖
deep_robotics_model/ 固定版本的 DR02 Pro 模型（Git submodule）
```

## 获取仓库

克隆本仓库时同时初始化模型 submodule：

```bash
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

如果已经通过普通 `git clone` 获取本仓库，则进入仓库根目录后执行：

```bash
git submodule update --init --recursive
```

> [!NOTE]
>
> GitHub 的 **Download ZIP** 不会初始化 submodule。需要完整运行环境时，请使用上述 Git
> 克隆方式。

## 从这里开始

请先阅读 [DR02 Pro Pico 遥操使用指南](src/dr02_pro/README_CN.md)，再根据目标选择真机或仿真
流程。控制实机时不得使用仿真编译配置。

## 相关仓库

- [deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)：
  ROS 2 消息接口库，软件包名为 `drdds`。
- [deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model)：
  遥操 IK 使用的 DR02 Pro URDF 和网格模型；本仓库通过 submodule 固定其已验证版本。
- [deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation)：
  MuJoCo 仿真环境；仿真器不包含在本仓库中。

> [!WARNING]
>
> 真机遥操会直接发布运动指令。将本软件包连接到机器人前，必须先阅读真机安全要求。
