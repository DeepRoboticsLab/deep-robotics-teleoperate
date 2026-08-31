# DR02 Pro 仿真遥操

[English](SIMULATION.md) | **简体中文** ·
[返回 DR02 Pro Pico 遥操使用指南](../README_CN.md)

本文档给出 DR02 Pro Pico 仿真遥操的完整 SDK 侧流程。MuJoCo 仿真器不包含在本仓库中，
其安装和环境配置以
[deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation) 为准。

> [!IMPORTANT]
>
> SDK、Robotics Service 和仿真器是相互独立的进程，运行此仿真需要三个终端窗口。它们必须使用兼容的 ROS 2、`drdds`
> 和 `ROS_DOMAIN_ID` 环境，此程序默认执行在Ubuntu 22.04平台。

## 1. 准备 Pico 与 Robotics Service

### 1.1 在 Pico 安装 XRoboToolkit

从 [XRoboToolkit Unity Client Releases](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)
下载 Pico APK至本地开发计算机上。在头显中开启开发者模式和 USB 调试，然后在连接头显的计算机上安装：

```bash
sudo apt install android-tools-adb android-tools-fastboot
adb devices
adb install -g XRoboToolkit-PICO-x.x.x.apk
```

`adb devices` 必须显示 `device`。若显示 `unauthorized`，请在头显中确认 USB 调试授权。

### 1.2 在本地开发计算机上安装 Robotics Service

仿真默认运行在 Ubuntu 22.04 x86（amd64）开发计算机上。打开
[XRoboToolkit PC Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)，
下载 `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`。进入安装包所在目录并执行：

```bash
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

安装成功后，`/opt/apps` 下会生成 `roboticsservice` 文件夹：

```text
/opt/apps/roboticsservice/
```

Pico 必须与本地开发计算机连接到同一个 WiFi，并将接收地址设置为这台仿真主机在该 WiFi 下的
IP 地址，因为 Robotics Service 和 `state_machine` 都在该主机运行。

## 2. 下载项目

仿真遥操需要以下三个仓库：

- [DeepRoboticsLab/deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)
- [DeepRoboticsLab/deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation)
- [DeepRoboticsLab/deep-robotics-teleoperate](https://github.com/DeepRoboticsLab/deep-robotics-teleoperate)

在开发计算机上创建统一的项目目录，并将三个仓库并列克隆到该目录中：

```bash
cd ~/your_project
git clone https://github.com/DeepRoboticsLab/deep-robotics-msg.git
git clone https://github.com/DeepRoboticsLab/deep-robotics-simulation.git
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

下载完成后的目录结构如下：

```text
your_project/
├── deep-robotics-msg/
├── deep-robotics-simulation/
└── deep-robotics-teleoperate/
    └── deep_robotics_model/  # Git submodule
```

三个仓库应保持为同级目录，不要将其中一个仓库克隆到另一个仓库内部。

## 3. 检查 DR02 Pro 模型 submodule

`deep_robotics_model` 已作为 Git submodule 固定在遥操项目验证过的版本。使用第 2 节中的
`--recurse-submodules` 克隆后，模型会自动下载到遥操项目根目录。默认 IK 模型路径为：

```text
deep-robotics-teleoperate/deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

如果此前使用普通 `git clone` 下载了遥操项目，或者 submodule 目录为空，请执行：

```bash
cd ~/your_project/deep-robotics-teleoperate
git submodule update --init --recursive
```

确认默认 URDF 已经存在：

```bash
ls deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

正常使用时无需单独克隆模型仓库，也不需要设置环境变量。如果已经将经过验证的
`deep_robotics_model` 下载或安装到其他位置，则在启动 `state_machine` 的同一终端中，将环境
变量指向实际的 URDF 文件：

```bash
export DR02_PRO_TELEOP_URDF=/absolute/path/to/deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

设置该环境变量后，程序将优先使用指定的 URDF，而不再读取项目根目录下的默认路径。

初始化完成后，与本流程相关的遥操项目主要目录结构如下：

```text
deep-robotics-teleoperate/
├── src/
├── third_party/
├── deep_robotics_model/  # Git submodule
├── .gitmodules
├── README.md
├── README_CN.md
└── LICENSE
```

## 4. 编译仿真版本

### 4.1 编译并加载消息接口

先加载 ROS 2 环境，再进入第 2 节下载的
[deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg) 仓库。本地目录名为
`deep-robotics-msg`：

```bash
source /opt/ros/<ros-distro>/setup.bash
cd ~/your_project/deep-robotics-msg
```

按照该仓库 README 的说明完成编译。编译成功后，在同一终端中加载生成的消息接口环境：

```bash
source install/setup.bash
```

### 4.2 编译 `dr02_tele`

保持当前终端中的 ROS 2 和消息接口环境，进入遥操项目并编译仿真版本：

```bash
cd ~/your_project/deep-robotics-teleoperate
colcon build --packages-up-to dr02_tele \
  --cmake-args -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON
source install/setup.bash
```

如果改用新终端编译 `dr02_tele`，必须先按以下顺序重新加载环境：

```bash
source /opt/ros/<ros-distro>/setup.bash
source ~/your_project/deep-robotics-msg/install/setup.bash
```

`BUILD_SIM=ON` 只会关闭真机使用的运行时 `SafeController` 监控，不会启动仿真器，也不会修改
ROS Topic 或 Pico 控制逻辑。

> [!WARNING]
>
> 此编译结果仅用于仿真，不得用于控制实机。

### 4.3 配置 `deep-robotics-simulation`

新开一个终端，进入第 2 节下载的仿真仓库：

```bash
cd ~/your_project/deep-robotics-simulation
```

按照该仓库 README 的说明完成编译和环境配置。仿真仓库的配置及启动方式可能会更新，应以仿真仓库 README 为准。

## 5. 启动完整仿真会话

仿真器和状态机必须设置相同的 `ROS_DOMAIN_ID`。本流程统一使用示例值 `10`，因此下列所有
相关终端中的 `export ROS_DOMAIN_ID=10` 必须保持一致。`10` 本身没有特殊含义；如需改用其他
值，应在所有相关终端中同时修改。

然后按顺序启动：

1. 在仿真主机的第一个终端启动 Robotics Service：

   ```bash
   cd /opt/apps/roboticsservice/
   ./runService.sh
   ```

2. 在 Pico 中先打开 **设置** → **WiFi**，连接到与本地开发计算机相同的 WiFi。然后打开
   **资源库** → **未知来源** → **XRoboToolkit**，点击 **Enter**。将接收地址设置为仿真主机
   在该 WiFi 下的 IP 地址，连接成功后打开 **Head**、**Controller**、**Send**，并保持头显
   常亮。

3. 在第二个终端进入外部仿真仓库并启动 DR02 Pro 仿真器。以下是当前使用的启动形式；若仿真
   仓库说明发生变化，以其文档为准：

   ```bash
   cd ~/your_project/deep-robotics-simulation
   export ROS_DOMAIN_ID=10
   source /opt/ros/<ros-distro>/setup.bash
   source ~/your_project/deep-robotics-msg/install/setup.bash
   source .venv/bin/activate
   python3 run_sim.py dr02_pro
   ```

4. （如果需要）可以在一个新的终端确认仿真器正在持续发布关节反馈：

   ```bash
   export ROS_DOMAIN_ID=10
   source /opt/ros/<ros-distro>/setup.bash
   source ~/your_project/deep-robotics-msg/install/setup.bash
   ros2 topic hz /JOINTS_DATA
   ```

   观察数个采样结果后按 Ctrl+C 结束检查命令。

5. 在第三个终端启动状态机：

   ```bash
   cd ~/your_project/deep-robotics-teleoperate
   export ROS_DOMAIN_ID=10
   source /opt/ros/<ros-distro>/setup.bash
   source ~/your_project/deep-robotics-msg/install/setup.bash
   source install/setup.bash
   ros2 run dr02_tele state_machine
   ```

6. 等待 `/JOINTS_DATA` 首帧和控制提示。在状态机终端按键盘 `z` 进入 `ZeroPos`，等待三秒
   过渡完成后，按键盘 `s` 进入 `Teleoperation`。
7. 等待终端出现 `PXREA server connected`、`device found` 和第一帧 Pico 数据提示。捏住单侧
   Pico 手柄的 trigger（可以捏到底），然后小幅移动该手柄开始验证。

默认命令不会创建夹爪 Topic。若要检查知行夹爪指令链路，请使用
`ros2 run dr02_tele state_machine --ee zhixing` 启动。当前仿真器尚未显示夹爪动作，可在另一个
已加载相同 ROS 2 环境和 `ROS_DOMAIN_ID` 的终端中检查：

```bash
ros2 topic echo /gripper/position_cmd
```

进入 `Teleoperation` 时不会自动发送张开指令。进入后第一包新鲜 grip 到达时才发布第一条消息，
消息格式为 `[2, left, right]`，其中 `2` 表示双侧，左、右位置随对应 grip 在 `0` 至
`1000` 之间变化。仿真环境没有 RK 侧 SDK 发布 `/gripper/position` 时，16 维录制不会开始。

状态切换、Pico 控制和诊断输出参阅[状态机与 Pico 控制](STATE_MACHINE_CN.md)。

## 6. 结束仿真会话

1. 松开左右 trigger。
2. 在状态机终端按键盘 `d` 进入 `Down`，等待三秒过渡完成。
3. 在状态机终端按 Ctrl+C。
4. 停止仿真器和 Robotics Service。

需要进入软件 `Emergency` 姿态保持时，在状态机终端按键盘 `e`。

## 7. 障碍排除

| 现象 | 优先检查 |
| --- | --- |
| `state_machine` 一直等待数据 | 仿真器是否持续发布 `/JOINTS_DATA`，以及所有终端的 `ROS_DOMAIN_ID` |
| 仿真器收不到指令 | `/JOINTS_CMD`、状态机反馈和 ROS 2/DDS 环境；仅 `--ee zhixing` 时检查 `/gripper/position_cmd` |
| Grip 没有显示动作 | 当前仿真器尚不显示夹爪动作；使用 `ros2 topic echo /gripper/position_cmd` 检查指令 |
| PXREA 初始化失败 | 是否使用 `BUILD_PLATFORM=x86`，以及 Robotics Service 是否正常运行 |
| `packets=0` | Pico 接收 IP、**Send** 开关和 Robotics Service |
| `pico_age_ms > 200` | 头显是否休眠、发送开关和 Pico 到主机的网络 |
| IK 初始化失败 | DR02 Pro URDF 路径及模型版本 |

> [!NOTE]
>
> 仿真可以验证遥操指令链路，但不能替代真机的开发者模式、工作空间和硬件急停检查。
