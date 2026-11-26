# DR02 Pro 三相机数据采集

[English](RECORDING.md) | **简体中文** ·
[返回 DR02 Pro Pico 遥操作指南](../README_CN.md)

本文介绍集成在 `state_machine` 中的三相机 RGB 数据采集功能。该功能不启动独立进程，
也不新增 ROS 2/DDS 录制 Topic。启用数据采集时，建议在 PICO 头显中关闭 **Head**。

> [!IMPORTANT]
>
> 数据采集的首要前提是已经按照[真机遥操作指南](REAL_ROBOT_CN.md)跑通完整的真机遥操作
> 流程。首次使用时，应先不带 `--record` 启动 `state_machine`，确认机器人状态切换、Pico
> 双臂遥操作正常；如需采集夹爪数据，再使用 `--ee zhixing` 验证夹爪控制。随后检查并配置
> 三台 RealSense 相机，最后加入 `--record`。数据采集不是一套独立启动流程。

## 启动前配置

首先确认机器人 Orin 已安装 RealSense SDK（librealsense）。如未安装，请参考
[deep-robotics-sdk2 的 RealSense 相机指南](https://github.com/DeepRoboticsLab/deep-robotics-sdk2/blob/main/src/dr02_pro/docs/CAMERA_CN.md)。

执行以下命令查看当前连接的 RealSense 设备及其序列号：

```bash
rs-enumerate-devices -s
```

如果无法确定两台 D405 分别对应左腕还是右腕，可以在建立 SSH 连接时启用 X11 转发：

```bash
ssh -Y user@10.21.33.106
realsense-viewer
```

在 `realsense-viewer` 中逐台查看图像并确认安装位置。确认后关闭
`realsense-viewer`，因为它不能与录制程序同时占用同一台相机。

按照“左腕、右腕、头部”的顺序，将确认后的序列号填写到
`src/dr02_pro/state_machine/src/recording_manager.cpp` 中的
`kCameraSerials`，然后按照[真机遥操作指南](REAL_ROBOT_CN.md)重新编译。

## 启动

启用数据采集并使用默认目录：

```bash
ros2 run dr02_tele state_machine --record
```

默认数据目录为启动命令所在目录下的 `record_data/`。程序会在启动时将其转换为绝对路径
并打印。

指定数据目录：

```bash
ros2 run dr02_tele state_machine --record \
  --record-dir /absolute/path/to/record_data
```

`--record-dir` 必须与 `--record` 一起使用。不带 `--record` 时，程序不会初始化相机、
同步器或 HDF5 写盘线程。

`--record` 单独使用时录制 14 维双臂数据；需要知行夹爪控制和 16 维数据时使用：

```bash
ros2 run dr02_tele state_machine --ee zhixing --record
```

## 录制操作

1. 按照正常状态机流程进入 `ZeroPos`。
2. 按 `s` 或机器人手柄 L2 进入 `Teleoperation`。
3. 确认终端显示三台相机均已在线。
4. 按 PICO 左手柄主键开始一个 episode。
5. 完成动作后再次按左手柄主键，正常结束并保存。
6. 未录制时按 PICO 左手柄副键，可直接删除最近的 `episode_N.hdf5`。

数据采集按键仅在 `Teleoperation` 中有效。进入遥操作前已经按住的按键不会触发操作，
必须先松开再重新按下。

Trigger 和 grip 均不控制录制状态：

- 松开 trigger 后，机械臂保持最近的安全目标，但图像、状态和动作继续写入。
- `--ee zhixing` 模式下，松开 grip 只改变对应夹爪目标，不会结束录制。
- 正常结束只能在遥操作模式中再次按下左手柄主键。

录制过程中进入 `Down`、`Emergency`、触发安全保护或按 `Ctrl+C` 时，当前未完成的
episode 会被关闭并删除，不会生成最终 HDF5 文件。

使用 `--ee zhixing --record` 时，开始 episode 还要求夹爪反馈新鲜有效；反馈缺失时拒绝开始，
录制中反馈超过 `200 ms` 未更新则中止并删除当前 episode，但双臂遥操作继续。

## 相机配置

默认相机位置和采集配置如下；序列号应按现场设备填写，不在文档中固化真实设备号。

| 相机 | 序列号 | 配置 |
| --- | --- | --- |
| 左腕 D405 | `<left-wrist-d405-serial>` | RGB8，640 × 480，30 FPS |
| 右腕 D405 | `<right-wrist-d405-serial>` | RGB8，640 × 480，30 FPS |
| 头部 D435 | `<head-d435-serial>` | RGB8，640 × 480，30 FPS |

只有三路相机全部在线时才允许开始录制，程序不会使用黑图占位。三路图像允许的最大同步偏差
为 20 ms；超出该范围时，程序会丢弃时间最早的一路帧并重新同步。

任一相机连续两秒没有有效帧时会被判定为掉线：

- 未录制时拒绝开始新的 episode，遥操作继续运行。
- 录制中自动删除当前未完成的 episode。
- 后台每两秒重试一次，三路相机恢复后重新允许录制。

如果当前构建未检测到 RealSense SDK，程序仍可正常执行遥操作，但使用 `--record` 时会提示
数据采集不可用。

## 数据目录

```text
record_data/
├── episode_0.hdf5
├── episode_1.hdf5
└── ...
```

录制期间先写入临时文件：

```text
episode_N.hdf5.tmp
```

正常结束并完成 flush/close 后，程序才会在同一目录中将其原子重命名为
`episode_N.hdf5`。零帧、相机掉线、写盘失败或离开遥操作状态都会导致临时文件被删除；
程序启动时也会清理上次异常退出遗留的临时文件。

删除最近一个 episode 后，其编号会被下一次采集复用。该删除操作直接作用于文件系统，
不提供恢复功能。

## HDF5 格式

每一行包含一组三路同步图像，以及时间最接近的一帧机器人状态和动作。目标保存频率约为
30 Hz。

不带末端执行器时为 14 维：

```text
左臂 7、右臂 7
```

使用 `--ee zhixing` 时为 16 维：

```text
左臂 7、右臂 7、左夹爪 1、右夹爪 1
```

当前数据协议不记录腰部。双臂 `qpos` 使用关节实测位置，双臂 `action` 使用实际发布的
滤波后位置目标。夹爪按照 CR1 约定保存为：

```text
qpos   = 1 - clamp(hand_feedback / 1000, 0, 1)
action = 1 - clamp(grip, 0, 1)
```

夹爪 `qpos` 只使用有效实测反馈，不再由 `action` 回填。

```text
episode_N.hdf5
├── attributes
│   ├── fps = 30
│   ├── state_fps = 30
│   ├── state_dimension = D
│   ├── state_layout
│   ├── end_effector = "none" 或 "zhixing"
│   ├── episode_len
│   ├── result = "success"
│   ├── sim
│   ├── depth_enabled = false
│   ├── state_image_match_policy
│   ├── image_timestamp_source = "host_arrival_wall_time"
│   ├── camera_sync_tolerance_ms = 20
│   ├── max_camera_sync_skew_ms
│   └── max_abs_state_image_offset_ms
├── actions                         float64 [N,D]
└── observations/
    ├── qpos                        float64 [N,D]
    ├── control_mode                float64 [N]，恒为 1
    ├── gate                        float64 [N]，恒为 0
    ├── timestamps                  float64 [N]
    ├── state_timestamps            float64 [N]
    ├── state_time_offset_ms        float64 [N]
    ├── camera_sync_skew_ms         float64 [N]
    ├── image_timestamps/
    │   ├── cam_d405_left           float64 [N]
    │   ├── cam_d405_right          float64 [N]
    │   └── cam_d435                float64 [N]
    └── images/
        ├── cam_d405_left            uint8 [N,500000]
        ├── cam_d405_left_len        uint64 [N]
        ├── cam_d405_right           uint8 [N,500000]
        ├── cam_d405_right_len       uint64 [N]
        ├── cam_d435                 uint8 [N,500000]
        └── cam_d435_len             uint64 [N]
```

`timestamps` 取三路相机时间戳中的最小值；`camera_sync_skew_ms` 为同一行三路时间戳的
最大值与最小值之差；`state_time_offset_ms = state_timestamps - timestamps`。根属性中的
两个 `max_*` 字段保存整条 episode 对应绝对偏差的最大值。

相机到达时间和机器人状态时间戳均为主机系统时钟自 Unix epoch 起的毫秒数，不是控制计时
使用的单调时钟。主机日期停在 1970 年时，保存的时间戳也会反映该日期；episode 内不要调整
系统时钟，因为同步过程依赖这些时间戳。

RGB 图像以 JPEG 质量 95 编码。每个图像数据集使用固定的 500000 字节行长，`*_len` 保存
对应 JPEG 的实际长度。图像数据集启用 HDF5 `deflate(1)` 以压缩行末的零填充；该设置不会
再次改变 JPEG 画质，也不会改变现有 schema。读取时应先按照 `*_len` 截取数据，再进行
JPEG 解码。

## 真机采集前检查

开始正式采集前，请确认：

1. 三台相机序列号与现场配置一致，并且全部显示为 online。
2. `/JOINTS_DATA` 持续更新；16 维模式还需确认 RK SDK 发布的
   `/gripper/position` 中左右两侧反馈均持续更新。
3. 按左手柄主键后，终端显示 `episode N started`。
4. 松开 trigger 后 recorder 仍处于 recording；知行模式下松开 grip 也不结束录制。
5. 再次按左手柄主键后，目录中仅保留最终的 `.hdf5`，不存在 `.tmp`。
6. 使用 HDF5 工具确认所有数据集的第一维均等于 `episode_len`，并抽样解码三路 JPEG。
