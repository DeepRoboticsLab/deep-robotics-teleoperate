# DR02 Pro Pico 遥操使用指南

[English](README.md) | **简体中文** ·
[返回仓库总览](../../README_CN.md)

`dr02_tele` 是 DR02 Pro Pico 上半身遥操软件包，唯一的运行程序是 `state_machine`。该程序
连接三条数据链路：

- XRoboToolkit/PXREA 提供 Pico 头显和手柄数据。
- ROS 2/DDS 提供实机或仿真器的关节反馈。
- 状态机通过 `/JOINTS_CMD` 发布位置 PD 目标。


## 运行组件

| 组件 | 运行位置 | 作用 |
| --- | --- | --- |
| XRoboToolkit | Pico 头显 | 发送头显姿态、手柄姿态、按键和 trigger 数据。 |
| Robotics Service | 遥操运行主机 | 接收 Pico 数据，并提供本机 PXREA 服务端点。 |
| `state_machine` | 与 Robotics Service 相同的主机 | 接收 PXREA 和 ROS 2 数据、计算目标并发布指令。 |
| DR02 Pro 或仿真器 | ROS 2/DDS 网络 | 发布反馈并接收关节指令。 |

仿真遥操运行在 x86 仿真主机，真机遥操运行在机器人 Orin（`10.21.33.106`）。Pico 的接收地址始终指向运行 Robotics Service 和 `state_machine` 的主机。

## 选择使用流程

| 目标 | 使用文档 | 编译配置 |
| --- | --- | --- |
| 仿真遥操 | [仿真遥操](docs/SIMULATION_CN.md) | x86 主机，`BUILD_PLATFORM=x86`、`BUILD_SIM=ON` |
| 真机遥操 | [真机遥操](docs/REAL_ROBOT_CN.md) | 机器人 Orin，`BUILD_PLATFORM=arm`、`BUILD_SIM=OFF` |

完成所选流程后，可将[状态机与 Pico 控制](docs/STATE_MACHINE_CN.md)作为状态切换、Pico 行为、
保护机制、诊断和排障的参考文档。

## 目录结构

- `manipulation/`：PXREA 接收、Pico 到机器人的映射、滤波和双臂 IK
- `state_machine/`：ROS 2 接口、输入处理、运行时安全监控和 `state_machine` 可执行程序
- `docs/`：完整的真机、仿真流程和状态机参考文档

> [!WARNING]
>
> - 同一时刻只能运行一个 `/JOINTS_CMD` 发布者。
> - `Emergency` 是软件姿态保持，不是硬件急停。
> - 首次控制实机前应先在仿真中验证完整流程；真机验证必须清空工作空间并从小幅动作开始。
