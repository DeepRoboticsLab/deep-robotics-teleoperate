# DR02 Pro 状态机与 Pico 控制

[English](STATE_MACHINE.md) | **简体中文** ·
[返回 DR02 Pro Pico 遥操使用指南](../README_CN.md)

本文档是 `dr02_tele` 的运行参考，说明 `state_machine` 的接口、状态流转、Pico 映射、指令
保护和诊断。真机的环境准备与完整启动步骤参阅[真机遥操](REAL_ROBOT_CN.md)；仿真的环境准备与
完整启动步骤参阅[仿真遥操](SIMULATION_CN.md)。

## 1. 控制循环与 ROS 2 接口

`state_machine` 创建名为 `sdk_deploy` 的 ROS 2 节点。主循环每 2 ms 唤醒一次，但只有
`/JOINTS_DATA` 的本地接收时间戳发生变化时才执行一次状态控制并发布新指令。因此实际控制频率
由关节反馈频率决定，不是固定的 500 Hz。

| 接口 | 方向 | 用途 |
| --- | --- | --- |
| `/JOINTS_DATA` | 订阅 | 31 自由度关节位置、速度、力矩、温度和状态反馈；每个新帧触发一次控制更新。 |
| `/JOINTS_CMD` | 发布 | 完整的 31 自由度位置 PD 指令。 |
| `/gripper/position` | 订阅 | RK 侧新知行 SDK 的反馈 `[side, position]`；仅使用 `--ee zhixing` 时创建。 |
| `/gripper/position_cmd` | 发布 | 新知行 SDK 的双侧指令 `[2, left, right]`；首包新鲜 grip 到达后发布，之后每 20 个控制更新发布一次。 |
| `/GAMEPAD_KEY` | 订阅 | 可选状态切换输入，识别 L1、L2、R1、R2 对应字符串。 |

启动时如果出现 `wait data update` 表示状态机还没有收到第一帧 `/JOINTS_DATA`。首帧到达前，程序
不会进入初始状态，也不会发布 `/JOINTS_CMD`。不带 `--ee zhixing` 时，程序在整个生命周期中
都不会创建或发布 `/gripper/position_cmd`。

## 2. 关节控制归属

| 关节索引 | 部位 | `Teleoperation` 中的目标来源 |
| --- | --- | --- |
| `0-2` | 腰部 | 头显相对姿态。 |
| `3-9` | 左臂 | 左 Pico 手柄的 6D 目标经过双臂 IK 得到。 |
| `10-16` | 右臂 | 右 Pico 手柄的 6D 目标经过双臂 IK 得到。 |
| `17-28` | 双腿 | 保持进入状态时记录的目标；Pico 不更新。 |
| `29-30` | 颈部 | 保持进入状态时记录的目标；Pico 不更新。 |

状态机始终构造 31 关节指令。实机最终采用哪些关节目标，还取决于机器人当前开发者模式；本项目
只允许配合已批准的上半身关节控制开发者模式使用。

可选末端执行器不属于上述 31 个关节。使用 `--ee zhixing` 时，Orin 发布
`std_msgs/msg/Int32MultiArray` 指令 `[2, left, right]` 到
`/gripper/position_cmd`，其中 `2` 表示双侧，后两项分别是左、右夹爪位置。RK 侧 SDK
通过 `/gripper/position` 分别返回 `[0, left_position]` 和
`[1, right_position]`。

## 3. 状态流转

```mermaid
stateDiagram-v2
    [*] --> Emergency
    Emergency --> ZeroPos: z / L1
    Emergency --> Down: d / R2
    ZeroPos --> Teleoperation: s / L2，且过渡已完成
    ZeroPos --> Emergency: e / R1
    Teleoperation --> Down: d / R2
    Teleoperation --> Emergency: e / R1
    Down --> ZeroPos: z / L1
    Down --> Emergency: e / R1
```

| 状态 | 指令行为 |
| --- | --- |
| `Emergency` | 记录进入该状态时的反馈位置，并使用位置 PD 保持该姿态。它不是断电、阻尼或硬件急停。 |
| `ZeroPos` | 使用三次样条在三秒内将 31 个关节指令插值到零位，然后保持零位。 |
| `Teleoperation` | 初始化 Pico/PXREA，以进入时姿态为初始目标，更新腰部和双臂；若已选择末端执行器则同时更新末端执行器，并保持腿部与颈部目标。 |
| `Down` | 在三秒内将腰部和双臂插值到下垂姿态，同时保持进入该状态时的腿部与颈部目标。 |


## 4. 键盘和机器人手柄输入

| 输入 | 有效状态 | 结果 |
| --- | --- | --- |
| `z` / L1 | `Emergency`、`Down` | 请求进入 `ZeroPos`。 |
| `s` / L2 | `ZeroPos` | 请求进入 `Teleoperation`；若三秒回零尚未结束，状态机会在结束后自动切换。 |
| `d` / R2 | `Emergency`、`Teleoperation` | 请求进入 `Down`。 |
| `e` / R1 | 任意状态 | 请求进入 `Emergency` 软件姿态保持。 |

键盘输入不区分大小写。机器人手柄通过 `/GAMEPAD_KEY` 发送 `G20_KEY_L1`、`G20_KEY_L2`、
`G20_KEY_R1`、`G20_KEY_R2`。如果在不允许的当前状态下按下对应按钮，状态机不会执行切换，
并会在终端输出忽略该请求的原因。

## 5. Pico 锚点与控制行为

进入 `Teleoperation` 时，状态机首先读取当前 31 关节反馈，初始化 IK、目标和滤波器，然后启动
PXREA。此时手臂不会因为 Pico 的绝对位置而直接跳转。

### 手柄控制双臂和可选夹爪

- 当某侧 trigger 达到 `0.5` 时，记录该手柄姿态和该侧机器人末端姿态作为锚点。
- trigger 按下后的第一帧只建立锚点，不求解位移；后续帧使用相对锚点的平移和旋转生成 6D
  末端目标。
- 松开 trigger 会清除该侧锚点。下次按下时从当前机器人指令姿态重新建锚，避免沿用旧目标跳变。
- 使用 `--ee zhixing` 时，左右 grip 独立于 trigger 控制对应夹爪；默认模式忽略 grip。
- 启用录制时，左手柄 primary 用于开始或结束 episode，secondary 用于删除最近一个已完成的
  episode；摇杆方向和摇杆按下数据当前不绑定控制功能。
- 左右手柄可以独立扣住，但双臂目标由同一次 IK 联合求解。

手柄映射参数：

| 参数 | 数值 |
| --- | ---: |
| Trigger 阈值 | `0.5` |
| 平移死区 | `0.015 m` |
| 平移比例 | `1.2` |
| 相对锚点最大平移 | `0.5 m` |
| 旋转死区 | `0.035 rad` |
| 相对锚点最大旋转 | `1.45 rad` |
| Grip 输入范围 | `[0, 1]` |
| 夹爪位置范围 | `[0, 1000]`，即 `position = grip × 1000` |
| 夹爪发送节拍 | 每 20 个状态机控制更新一次 |

平移和旋转死区采用连续映射：超过阈值后只映射超出死区的部分，避免目标在越过阈值时产生
位置或角度阶跃。

启用知行夹爪后，进入 `Teleoperation` 不会自动张开，也不会使用进入前缓存的 PICO 数据。
收到进入状态后的第一包新鲜且有效 grip 后立即发布第一条指令，之后按照上述节拍保持最近目标。
指令位置为整数 `[0,1000]`，由 RK 侧 SDK 转换为 CAN 寄存器操作。任意一侧夹爪反馈缺失或
超过 `200 ms` 时只对夹爪限频告警，双臂与腰部遥操作继续。由于 RK 与 Orin 时钟不同，反馈
新鲜度只按消息到达 Orin 时的本地单调时钟计算，不使用或比较 RK 时间。

### 头显控制腰部

进入 `Teleoperation` 后，第一份有效头显姿态会建立头显和腰部锚点。之后使用相对姿态控制：

| 头显变化 | 腰部目标 | 比例 |
| --- | --- | ---: |
| pitch | 关节 `0` | `+0.8` |
| yaw | 关节 `1` | `-0.9` |
| roll | 关节 `2` | `-0.6` |

头显角度死区为 `0.006 rad`，一阶滤波系数为 `alpha = 0.88`。腰部最终还会经过软件限位和
每反馈帧限速。

## 6. Pico 断流保护

状态机使用 Pico 数据在本机被接收时的单调时钟判断数据新鲜度，不把它当作网络延迟测量。

- 尚未收到有效 Pico 数据，或最近数据超过 `200 ms` 未更新时，进入 soft stale hold。
- soft stale hold 会冻结最近一次真正发送给机器人的目标，不会继续靠近尚未下发的旧 IK 目标，
  并清除手柄和头显锚点。
- 连续收到 `3` 个新 Pico 数据包后退出 soft stale；若 trigger 仍按住，则以当前手柄位置和机器人
  指令重新建立锚点，无需先松开 trigger。
- 数据超过 `1000 ms` 未更新时升级为 hard stale，恢复后要求所有 trigger 先松开。
- 已启用的夹爪沿用最近一次有效 grip 缓存，并继续按原节拍保持该目标；新 stale 数据不会改变目标。

只有进入 hard stale 后，恢复时仍扣住 trigger 才会继续保持手臂，直到 trigger 松开。

## 7. IK 与指令保护链路

IK 在进入 `Teleoperation` 时加载默认 URDF，或加载 `DR02_PRO_TELEOP_URDF` 指定的文件。
它在完整 31 自由度模型中锁定腰部、颈部和双腿的 17 个关节，只保留双臂 14 自由度。

每份新的 Pico 状态最多触发一次双臂联合 IK，每次求解最多执行 50 次内部迭代；更高频的关节反馈
仍持续执行滤波和指令下发。保护和整形按实际执行顺序分为：

1. 将手柄相对平移和旋转限制在锚点附近的允许范围。
2. IK 同时优化双臂位置与姿态，并在肘部进入躯干半径 `0.25 m` 的圆柱区域时加入权重 `5.0`
   的向外软代价。关节姿态正则权重为 `0.002`，相对种子的连续性正则权重为 `0.02`。
3. 每次内部迭代使用 URDF 关节位置上下限裁剪候选，并只接受让总笛卡尔任务误差和完整目标函数
   都不增加的步长；完整步不满足时最多执行 6 次二分回退，并自适应增大阻尼。
4. 求解器保存历史最佳迭代，不会因为最后一轮数值振荡而返回更差结果。
5. IK 出口拒绝非有限值、相对种子任务误差变差、任一末端位置残差超过 `0.05 m` 或姿态残差
   超过 `15 deg` 的候选。拒绝时保持最近一次真正下发的手臂指令，并清零滤波器速度。
6. 对通过检查的候选，将每个关节的原始目标相对当前已下发命令的单次推进量限制为
   `0.5 rad`。更大的合法解会被裁剪后继续进入平滑器，而不是整份拒绝，避免快速运动时目标
   锁死在旧命令。
7. 14 维手臂目标经过四帧加权平均 `{0.1, 0.2, 0.3, 0.4}` 和临界阻尼二阶平滑器
   `omega = 30`，并同时下发平滑后的双臂位置和速度。
8. 腰部和双臂目标经过下表的软件限位。
9. 最终输出相对上一条已发送指令的变化量限制为每反馈帧 `0.006 rad`。

| 索引 | 关节 | 软件范围（rad） |
| ---: | --- | --- |
| `0` | waist z | `[-0.6, 0.6]` |
| `1` | waist x | `[-0.4, 0.4]` |
| `2` | waist y | `[-0.4, 0.4]` |
| `3` / `10` | left / right shoulder y | `[-1.2, 1.4]` |
| `4` | left shoulder x | `[-0.7, 1.8]` |
| `11` | right shoulder x | `[-1.8, 0.7]` |
| `5` / `12` | left / right shoulder z | `[-1.6, 1.6]` |
| `6` / `13` | left / right elbow | `[-0.2, 1.8]` |
| `7` / `14` | left / right wrist z | `[-2.6, 2.6]` |
| `8` / `15` | left / right wrist y | `[-1.45, 1.45]` |
| `9` / `16` | left / right wrist x | `[-1.45, 1.45]` |

肘部圆柱代价是软约束，不是网格碰撞检测。它不能检测双臂互撞，也不能保护机器人免于环境或
人员碰撞。

## 8. 真机运行安全监控

`BUILD_SIM=OFF` 时，`SafeController` 启动独立监控线程，线程周期配置为 `2 ms`。当前代码执行
以下检查：

- 检查关节位置、速度和力矩是否为有限值。收到首个关节更新后，如果关节反馈时间戳超过
  `100 ms` 未变化，则判定异常并设置 `safe_control_mode=3`。只要完整反馈帧持续到达，关节位置、
  速度和力矩保持不变不会被判定为异常。
- 每 1000 个监控周期检查一次电机最高温度。最高温度高于 `110 °C` 时触发保护。

达到异常条件的检查会将 `safe_control_mode` 设置为非零。状态机在反馈更新门控之前检查该值，
因此即使关节反馈完全停止，也会退出当前控制状态并立即进入 `Emergency`。没有新反馈时不会继续
生成或发布关节指令，真机端仍必须配置独立的驱动或硬件 watchdog。安全监控不输出周期或异常
诊断信息。


## 9. 运行诊断与排障

默认不输出周期诊断。使用 `ros2 run dr02_tele state_machine --log`
启动后，`Teleoperation` 每秒输出一次诊断：

```text
[PicoTeleoperation] diag pico_age_ms=... packets=...(total=...)
parse_fail=...(total=...) feedback_hz=... ik_hz=...
ik_reject=...(total=...) ik_limit=...(total=...)
ik_ms(avg/max)=.../...
control_ms(avg/max)=.../... trigger(L/R)=.../... stale=...
```

| 字段或现象 | 含义与检查方向 |
| --- | --- |
| `pico_age_ms` | 最近有效 Pico 数据的本机接收时间；超过 200 表示断流保护。 |
| `packets` | 最近一秒收到的有效状态包；为 0 时检查 Pico **Send** 和 Robotics Service。 |
| `parse_fail` | JSON 解析失败数量；持续增加时检查发送端数据格式和版本。 |
| `feedback_hz` | 实际关节反馈与控制更新频率；过低时检查 `/JOINTS_DATA` 和 DDS。 |
| `ik_hz` | 实际 IK 求解频率；IK 只对新的 Pico 数据求解，不随更高的关节反馈频率重复计算。 |
| `ik_reject` | 最近一秒和累计被安全门拒绝的 IK 候选数；持续增加表示目标不可达、接近奇异位形或输入发生跳变。 |
| `ik_limit` | 每关节单次推进量被裁剪到 `0.5 rad` 的合法 IK 目标数量；快速运动时增长属于预期行为，不会重置平滑器。 |
| `ik_ms(avg/max)` | IK 平均/最大耗时；异常增大时检查不可达目标、URDF 和主机负载。 |
| `control_ms(avg/max)` | 整个 Teleoperation 控制与发布耗时。 |
| `trigger(L/R)` | 左右 trigger 原始值，用于检查是否达到 `0.5`。 |
| `stale` | 是否正在执行 Pico 断流保持。 |

常见启动日志：

| 日志 | 含义 |
| --- | --- |
| `PXREAInit returned 0` | PXREA 初始化调用成功。 |
| `PXREA server connected` | 状态机已连接本机 Robotics Service。 |
| `device found` | Robotics Service 已发现 Pico 设备。 |
| `first state JSON` | 已收到并解析第一份 Pico 状态。 |
| `trigger latched` | 该侧刚建立锚点，需继续移动手柄才产生相对目标。 |
| `reject IK candidate` | IK 候选未通过残差、任务代价或数值检查；手臂保持最近一次已下发指令。 |
| `limit IK target` | 合法 IK 解超过单次目标推进限幅，裁剪后继续执行而不是整份拒绝。 |
| `Pico data stale` | Pico 数据超过 200 ms 未更新，输出已冻结并等待 3 个新数据包。 |
| `Pico data hard stale` | Pico 数据超过 1000 ms 未更新，恢复后必须先松开 trigger。 |

## 10. 参数位置

- Pico 坐标映射、死区、比例、软件限位、滤波和断流阈值：
  `manipulation/src/pico_teleoperation.cpp`
- IK 权重、迭代次数和肘部软避让：`manipulation/src/dr2_arm_ik.cpp`
- 状态目标和 PD 参数：`state_machine/state/` 与
  `state_machine/include/control_parameters.hpp`

调参时一次只修改一个变量，先在仿真中记录诊断结果，再按完整真机安全流程重新验证。

> [!WARNING]
>
> `Emergency`、安全监控、Pico 断流保持、软件限位和肘部软代价都不能替代硬件急停与安全工作
> 空间。真机遥操期间必须始终确保实体急停或手柄红色停止按钮可立即触及。
