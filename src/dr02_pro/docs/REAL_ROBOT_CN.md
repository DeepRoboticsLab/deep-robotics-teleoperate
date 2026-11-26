# DR02 Pro 真机遥操

[English](REAL_ROBOT.md) | **简体中文** ·
[返回 DR02 Pro Pico 遥操使用指南](../README_CN.md)

真机遥操仅支持运行在机器人 Orin（`10.21.33.106`）上。Robotics Service 和
`state_machine` 都运行在该设备。Pico 到 Orin 支持 **WiFi 无线**或 **USB 有线**连接；
两种方式共用后续的编译、服务启动和遥操步骤。

> [!WARNING]
>
> 本程序会通过 `/JOINTS_CMD` 直接下发运动指令。真正开启遥操前必须确认机器人已进入指定的上半身
> 关节控制开发者模式、周围工作空间安全、没有其他 `/JOINTS_CMD` 发布者，并确保手柄红色
> 停止按钮或实体急停可以立即触及。

## 1. 登录机器人 Orin

机器人正常开机，本地开发计算机通过网线连接机器人后，确认能够访问 `10.21.33.106`，再通过 SSH 登录：

```bash
ping 10.21.33.106
ssh user@10.21.33.106
```

机器人 WiFi、SSH 用户名和密码以交付资料或技术支持提供的信息为准。

## 2. 准备 Pico 与 Robotics Service

### 2.1 在 Pico 安装 XRoboToolkit

从 [XRoboToolkit Unity Client Releases](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)
下载 Pico APK至本地开发计算机上。在头显中开启开发者模式和 USB 调试，然后在连接头显的计算机上安装：

```bash
sudo apt install android-tools-adb android-tools-fastboot
adb devices
adb install -g XRoboToolkit-PICO-x.x.x.apk
```

`adb devices` 必须显示 `device`。若显示 `unauthorized`，请在头显中确认 USB 调试授权。

### 2.2 在机器人 Orin 安装 Robotics Service

打开
[XRoboToolkit PC Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)，
下载 ARM64 headless 安装包 `XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb`，并将其传输到
`10.21.33.106` 对应的机器人 Orin。在 Orin 上进入安装包所在目录并执行：

```bash
sudo dpkg -i XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
```

安装成功后，`/opt/apps` 下会生成 `roboticsservice` 文件夹：

```text
/opt/apps/roboticsservice/
```

实际安装路径以安装包或机器人部署环境为准。同一台 Orin 上只能运行一个 Robotics Service
实例。

### 2.3 选择 Pico 连接方式

选择以下一种方式。这里配置的是 Pico 到 Orin 的链路；本地开发计算机仍按第 1 节通过
网线 SSH 登录 Orin。

| 方式 | 操作 |
| --- | --- |
| WiFi 无线 | 按[第 2.3.1 节](#231-wifi-无线连接)让 Pico 与 Orin 连接同一个 WiFi。 |
| USB 有线 | 按[第 2.3.2 节](#232-usb-有线连接)通过 Type-C 数据线连接，无需配置两者的同网 WiFi。 |

#### 2.3.1 WiFi 无线连接

登录机器人 Orin 后，在当前 SSH 终端中先查看网络设备状态：

```bash
# 重点检查 wlan0 的 STATE 和 CONNECTION
nmcli device status
```

如果 `wlan0` 已显示为 `connected`，且连接的是目标 WiFi，请跳过下面的连接命令，直接查询 IP。

如果尚未连接，请依次执行：

```bash
# 启用 WiFi
sudo nmcli radio wifi on

# 扫描附近的 WiFi，在 SSID 列中找到目标网络
nmcli device wifi list

# 将 <wifi_ssid> 替换为实际 SSID，并按提示输入密码
sudo nmcli --ask device wifi connect "<wifi_ssid>"

# 确认已连接到目标 WiFi
nmcli connection show --active
```

列表中同名的 SSID 表示多个无线接入点，直接使用 SSID 连接即可，无需指定 BSSID。

连接后查询 Orin 的 WiFi IP：

```bash
nmcli -g IP4.ADDRESS device show wlan0
```

若无线网卡名称不是 `wlan0`，将查询命令中的名称替换为 `nmcli device status` 显示的实际名称。
输出如 `192.168.1.20/24`，斜线前的地址就是 `<Orin-WiFi-IP>`。

让 Pico 连接同一个 WiFi，后续接收地址填写 `<Orin-WiFi-IP>`，不是有线登录地址
`10.21.33.106`。该 WiFi 需允许设备之间互相通信，关闭客户端隔离（AP isolation）。

#### 2.3.2 USB 有线连接

用一根支持数据传输的 **Type-C to Type-C 数据线**，将 Pico 的 Type-C 口连接到
**机器人背部的 Type-C 口**。Pico 保持开发者模式和 USB 调试开启。

以下命令在**机器人 Orin** 的 SSH 终端执行，不是在本地开发计算机上执行。Orin 上若没有
`adb`，先运行 `sudo apt install android-tools-adb`，然后检查设备：

```bash
adb devices
```

确认列表中 Pico 的状态为 `device`；若为 `unauthorized`，在头显中允许 USB 调试。
设备确认后继续后面的共用步骤，在第 7 节启动服务前执行 USB 端口转发。

## 3. 下载项目并加载环境

在机器人 Orin 上克隆本项目：

```bash
cd ~
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

加载机器人中的 ROS 2 和 `drdds` 消息接口环境：

```bash
source /opt/ros/<ros-distro>/setup.bash
source <drdds-workspace>/install/setup.bash
```

`deep-robotics-msg` 的 ROS 2 软件包名为 `drdds`。如果机器人环境已经通过系统安装提供该
软件包，请使用交付环境对应的 setup 文件；如果通过源码编译，则加载其工作空间的
`install/setup.bash`。如果机器人环境无法定位该软件包，则在机器人 Orin 上克隆
[DeepRoboticsLab/deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)，
然后按照仓库指引完成编译并加载生成的 `install/setup.bash`。

进入项目根目录：

```bash
cd ~/deep-robotics-teleoperate
```

## 4. 检查 DR02 Pro 模型 submodule

`deep_robotics_model` 已作为 Git submodule 固定在本项目验证过的版本。使用第 3 节中的
`--recurse-submodules` 克隆后，模型会自动下载到项目根目录。默认 IK 模型路径为：

```text
deep-robotics-teleoperate/deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

如果此前使用普通 `git clone` 下载了本项目，或者该目录为空，请在项目根目录初始化
submodule：

```bash
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

初始化完成后，与本流程相关的项目主要目录结构如下：

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

## 5. 编译真机版本

使用未激活 Conda 的终端，加载上文的 ROS 2 和消息接口环境。若当前已激活 Conda，先执行
`conda deactivate` 直到退出环境，再加载这些环境。在项目根目录执行：

```bash
colcon build --packages-up-to dr02_tele \
  --cmake-args -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

若之前的 CMake 配置选中了 Conda 库，在上述命令的 `--cmake-args` 前添加
`--cmake-clean-cache` 重新配置；Conda 与系统 OpenSSL 库混用可能导致链接符号缺失。
`deep_robotics_common` 提示未使用 `BUILD_PLATFORM`、`BUILD_SIM` 可以忽略，这些选项由
`dr02_tele` 使用。

> [!WARNING]
>
> 真机编译必须使用 `BUILD_PLATFORM=arm`，并保持 `BUILD_SIM=OFF`。启用
> `BUILD_SIM=ON` 会关闭真机运行时安全监控，禁止用于实机。

## 6. 进入上半身关节控制开发者模式

启动遥操程序前，通过机器人手柄选择并进入**上半身SDK模式**。进入模式前应清空胸前、头部和身体两侧的运动空间。

状态机会发布完整的 31 自由度关节指令，其中 Pico 用于控制腰部和双臂，腿部运动由机器人内部策略负责。机器人运动过程中不得切换开发者模式。

使用知行夹爪前，必须先在 RK 板启动 `zx_gripper_sdk`。

## 7. 启动 Robotics Service

编译完成后，开启终端 1，登录机器人 Orin：

```bash
ssh user@10.21.33.106
```

**仅 USB 有线方式**：确认 `adb devices` 中 Pico 为 `device`，在 Orin 上执行端口转发。
重新插拔数据线或重启设备后，需要重新执行：

```bash
adb reverse tcp:63901 tcp:63901
```

**两种方式均执行**以下命令启动服务：

```bash
cd /opt/apps/roboticsservice/
./runService.sh
```

保持该终端和 Robotics Service 持续运行。有线方式还需保持 USB 数据线连接。

## 8. 配置 Pico

先按第 7 节在机器人 Orin 上启动 Robotics Service，再按所用连接方式操作。

### 8.1 WiFi 无线方式

1. 打开 **设置** → **WiFi**，将 Pico 连接到与机器人 Orin 相同的 WiFi。
2. 打开 **资源库** → **未知来源** → **XRoboToolkit**。
3. 点击 **Enter**。
4. 将接收地址设置为第 2.3.1 节查询到的 `<Orin-WiFi-IP>`，然后连接。
5. 连接成功后勾选 **Controller**、**Send**；需要腰部控制时勾选 **Head**，不使用腰部控制时
   不勾选，并保持头显常亮。

### 8.2 USB 有线方式

保持 Type-C 数据线连接 Pico 和机器人背部的 Type-C 口，并确认已在 Orin 上按第 7 节执行
`adb reverse tcp:63901 tcp:63901`。

1. 打开 **资源库** → **未知来源** → **XRoboToolkit**。
2. 直接点击 **Reconnect**，无需填写接收地址；确认显示绿色 **working**。
3. 勾选 **Controller**、**Send**；需要腰部控制时勾选 **Head**，不使用腰部控制时不勾选，
   并保持头显常亮。

两种方式使用 `--record` 录制时，均建议关闭 **Head**；
**Controller** 和 **Send** 保持开启。

## 9. 启动状态机

可以直接复用第 5 节中在项目根目录完成编译的终端。该终端已经连接机器人 Orin，并且已经加载所需环境，因此直接运行：

```bash
ros2 run dr02_tele state_machine
```

该默认命令只启用双臂和腰部，不创建夹爪 Topic。需要知行夹爪时改为：

```bash
ros2 run dr02_tele state_machine --ee zhixing
```

如果编译终端已经关闭，或者希望在独立终端中运行状态机，则开启终端 2，连接
`10.21.33.106`，重新加载环境并运行：

```bash
ssh user@10.21.33.106
source /opt/ros/<ros-distro>/setup.bash
source <drdds-workspace>/install/setup.bash
cd ~/deep-robotics-teleoperate
source install/setup.bash
ros2 run dr02_tele state_machine
```

上述独立终端命令同样可按需要追加 `--ee zhixing`、`--record` 或同时追加两者。

状态机首先等待 `/JOINTS_DATA`。控制提示出现后：

1. 按 `z` 或机器人手柄 L1 进入 `ZeroPos`，等待三秒过渡完成。
2. 按 `s` 或机器人手柄 L2 进入 `Teleoperation`。
3. 等待终端出现 `PXREA server connected`、`device found` 和第一帧 Pico 数据提示。
4. 捏住单侧 Pico 手柄的 trigger（可以捏到底），然后只小幅移动该手柄开始验证；确认该侧控制
   正常后，再逐步验证另一侧手柄和头显控制。

具体状态流转、Pico 锚点行为和诊断信息参阅[状态机与 Pico 控制](STATE_MACHINE_CN.md)。

## 10. 正常退出与紧急处理

正常退出：

1. 松开左右 trigger。
2. 按 `d` 或机器人手柄 R2 进入 `Down`，等待三秒过渡完成。
3. 在 `state_machine` 终端按 Ctrl+C，确认程序退出。
4. 停止 Robotics Service。
5. 最后退出机器人开发者模式。

发生异常时，应优先使用手柄红色停止按钮或实体急停。键盘 `e` 和机器人手柄 R1 只会进入软件
`Emergency` 姿态保持，不能替代硬件急停。

## 11. 障碍排除

| 现象 | 优先检查 |
| --- | --- |
| SSH 无法连接 | 开发计算机与机器人网络、`10.21.33.106` 连通性和 SSH 凭据 |
| 一直显示 `wait data update` | `/JOINTS_DATA`、ROS 2 环境、`drdds` 和 DDS 配置 |
| PXREA 初始化或加载失败 | Robotics Service 是否运行，以及是否使用 ARM 版本依赖 |
| 找不到 URDF | 项目根目录中的 `deep_robotics_model/`，或 `DR02_PRO_TELEOP_URDF` 路径 |
| WiFi 方式 Pico 无数据 | Pico 与 Orin 是否连接同一 WiFi、接收地址是否为 `<Orin-WiFi-IP>`、发送开关和头显常亮状态 |
| USB 方式 Pico 无数据 | 数据线是否连接机器人背部 Type-C 口、Orin 上 `adb devices` 是否为 `device`、`adb reverse --list` 是否包含 63901、Robotics Service 是否运行，以及点击 **Reconnect** 后是否显示绿色 **working**、发送开关是否开启 |
| 有数据但手臂不动 | `Teleoperation` 状态、trigger 与断流恢复状态、关节反馈是否有限，以及初始双臂位置是否在所加载 URDF 限位内 |
| 持续出现 `reject IK candidate` | 对照日志残差与位置/姿态阈值，详见[IK 诊断](STATE_MACHINE_CN.md#9-运行诊断与排障) |

> [!WARNING]
>
> - 同一时刻只能运行一个 `/JOINTS_CMD` 发布者。
> - 首次动作必须小幅、单侧、分步骤验证。
> - 停止 SDK 后才能退出或切换开发者模式。
> - 软件安全监控、Pico 断流保持和 `Emergency` 都不能替代硬件急停。
