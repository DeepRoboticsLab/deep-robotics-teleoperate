# DR02 Pro Pico 遥操作

[English](README.md) | **简体中文** ·
[返回仓库总览](../../README_CN.md)

`dr02_tele` 提供 DR02 Pro 上半身遥操作程序 `state_machine`。PICO 默认控制腰部和双臂；
末端执行器默认关闭，必须通过运行参数显式启用。

## 选择运行方式

| 目标 | 平台 | 编译配置 | 完整指南 |
| --- | --- | --- | --- |
| MuJoCo 仿真 | x86_64 | `BUILD_PLATFORM=x86`、`BUILD_SIM=ON` | [仿真遥操作](docs/SIMULATION_CN.md) |
| DR02 Pro 实机 | ARM64 Orin | `BUILD_PLATFORM=arm`、`BUILD_SIM=OFF` | [真机遥操作](docs/REAL_ROBOT_CN.md) |

PICO 头显、Robotics Service、`state_machine` 和机器人或仿真器是相互独立的运行组件。
安装、网络、编译及启动顺序请直接按照对应的完整指南执行。

## 运行选项

```bash
ros2 run dr02_tele state_machine [--log] [--record] [--record-dir PATH] [--ee zhixing]
```

| 选项 | 作用 |
| --- | --- |
| `--log` | 每秒输出一次 PICO 与控制诊断信息。 |
| `--record` | 启用集成的三相机 HDF5 episode 录制。 |
| `--record-dir PATH` | 指定录制目录，必须与 `--record` 一起使用。 |
| `--ee zhixing` | 启用知行双夹爪；不带该参数时不会创建夹爪 Topic。 |

常用组合：

| 命令后缀 | 控制与录制布局 |
| --- | --- |
| 无 | 仅双臂和腰部，不录制 |
| `--record` | 双臂和腰部，录制 14 维双臂数据 |
| `--ee zhixing` | 双臂、腰部和知行夹爪，不录制 |
| `--ee zhixing --record` | 双臂、腰部和知行夹爪，录制 16 维数据 |

录制功能需要 RealSense 支持和三台已配置的相机。相机映射、录制操作及文件格式见
[三相机数据录制](docs/RECORDING_CN.md)。

## 控制方式

| 输入 | 动作 |
| --- | --- |
| `z` / L1 | 从 `Emergency` 或 `Down` 进入 `ZeroPos`。 |
| `s` / L2 | 从 `ZeroPos` 进入 `Teleoperation`。 |
| `d` / R2 | 从 `Emergency` 或 `Teleoperation` 进入 `Down`。 |
| `e` / R1 | 进入软件 `Emergency` 姿态保持。 |
| PICO trigger | 建立锚点并控制对应机械臂。 |
| PICO grip | 使用 `--ee zhixing` 时控制对应夹爪。 |

启用录制时，PICO 左手柄主键用于开始或结束 episode，副键用于删除最近一个已完成的
episode。

## ROS 2 接口

| Topic | 方向 | 作用 |
| --- | --- | --- |
| `/JOINTS_DATA` | 订阅 | 31 自由度关节反馈 |
| `/JOINTS_CMD` | 发布 | 31 自由度关节指令 |
| `/gripper/position` | 订阅 | RK 侧新知行 SDK 的夹爪反馈 `[side, position]`；仅`--ee zhixing` 模式存在 |
| `/gripper/position_cmd` | 发布 | 发往 RK 侧新知行 SDK 的双夹爪指令 `[2, left,right]`；仅 `--ee zhixing` 模式存在 |
| `/GAMEPAD_KEY` | 订阅 | 机器人手柄状态指令 |

同一机器人或仿真器上只能运行一个关节指令发布者。
使用知行夹爪前，必须先在 RK 板启动 `zx_gripper_sdk`。

## 安全行为

- 真机编译版本会监控关节反馈和电机温度。
- 关节数据非法或反馈中断超过 `100 ms` 时请求进入 `Emergency`；只要完整反馈帧持续
  到达，关节数值保持不变仍视为正常。
- PICO 数据过期后会停止更新目标，并要求按安全流程恢复。
- `Emergency` 是软件姿态保持，不能代替机器人物理急停和硬件保护。

首次控制实机前必须先完成仿真验证。状态流转、PICO 行为和故障排查见
[状态机与 PICO 控制](docs/STATE_MACHINE_CN.md)。

## 软件包结构

```text
src/
├── end_effector/         可选末端执行器接口、工厂和具体实现
└── dr02_pro/
    ├── manipulation/     PICO 数据接收、输入整形和双臂 IK
    ├── state_machine/    ROS 2 接口、状态控制、安全监控和数据录制
    └── docs/             仿真、真机、控制和录制指南
```
