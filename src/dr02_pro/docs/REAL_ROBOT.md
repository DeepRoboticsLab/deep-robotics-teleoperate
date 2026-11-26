# DR02 Pro Real-Robot Teleoperation

**English** | [简体中文](REAL_ROBOT_CN.md) ·
[Back to the DR02 Pro Pico Teleoperation Guide](../README.md)

Real-robot teleoperation is supported only on the robot Orin at
`10.21.33.106`. Robotics Service and `state_machine` both run on that device.
Pico can connect to the Orin over **WiFi** or a **USB cable**; both modes share
the build, service startup, and teleoperation steps below.

> [!WARNING]
>
> This program sends motion commands directly through `/JOINTS_CMD`. Before
> enabling teleoperation, confirm that the robot is in the designated
> upper-body joint-control Developer Mode, the surrounding workspace is clear,
> no other `/JOINTS_CMD` publisher is running, and the red stop button on the
> robot gamepad or a physical emergency stop is immediately accessible.

## 1. Log In to the Robot Orin

Start the robot normally and connect the local development computer to the
robot with an Ethernet cable. Confirm that `10.21.33.106` is reachable, then
log in through SSH:

```bash
ping 10.21.33.106
ssh user@10.21.33.106
```

Use the robot WiFi settings and SSH credentials supplied in the delivery
materials or by technical support.

## 2. Prepare Pico and Robotics Service

### 2.1 Install XRoboToolkit on Pico

Download the Pico APK from
[XRoboToolkit Unity Client Releases](https://github.com/XR-Robotics/XRoboToolkit-Unity-Client/releases)
to the local development computer. Enable Developer Mode and USB debugging on
the headset, then install the APK from the computer connected to it:

```bash
sudo apt install android-tools-adb android-tools-fastboot
adb devices
adb install -g XRoboToolkit-PICO-x.x.x.apk
```

`adb devices` must report `device`. If it reports `unauthorized`, approve the
USB debugging request in the headset.

### 2.2 Install Robotics Service on the Robot Orin

Open
[XRoboToolkit PC Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases),
download the ARM64 headless package
`XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb`, and transfer it to the
robot Orin at `10.21.33.106`. On the Orin, enter the directory containing the
package and run:

```bash
sudo dpkg -i XRoboToolkit-PC-Service-headless_1.0.0.0_arm64.deb
```

After a successful installation, the `roboticsservice` directory is available
under `/opt/apps`:

```text
/opt/apps/roboticsservice/
```

The actual path depends on the installation package or robot deployment
environment. Run only one Robotics Service instance on an Orin.

### 2.3 Choose the Pico Connection

Choose one mode below for the Pico-to-Orin link. The local development
computer still uses the Ethernet/SSH connection from Section 1 to access Orin.

| Mode | Setup |
| --- | --- |
| Wireless WiFi | Follow [Section 2.3.1](#231-wireless-wifi-connection) to connect Pico and Orin to the same WiFi. |
| Wired USB | Follow [Section 2.3.2](#232-wired-usb-connection) to connect them with a Type-C data cable; shared WiFi is not required for this link. |

#### 2.3.1 Wireless WiFi Connection

After logging in to the robot Orin, check the network devices in the current
SSH terminal:

```bash
# Check STATE and CONNECTION for wlan0
nmcli device status
```

If `wlan0` is already `connected` to the target WiFi, skip the connection
commands below and go directly to the IP query.

If it is not connected, run:

```bash
# Enable WiFi
sudo nmcli radio wifi on

# Scan nearby networks and find the target name in the SSID column
nmcli device wifi list

# Replace <wifi_ssid> with the actual SSID and enter the password when prompted
sudo nmcli --ask device wifi connect "<wifi_ssid>"

# Confirm the connection to the target WiFi
nmcli connection show --active
```

Repeated SSIDs in the list represent multiple access points. Connect by SSID;
there is no need to specify a BSSID.

Once connected, query the Orin's WiFi IP:

```bash
nmcli -g IP4.ADDRESS device show wlan0
```

If the wireless interface is not named `wlan0`, replace it in the query with
the actual name shown by `nmcli device status`. For output such as
`192.168.1.20/24`, the address before the slash is `<Orin-WiFi-IP>`.

Connect Pico to the same WiFi and use `<Orin-WiFi-IP>` as its receiver address,
rather than the wired login address `10.21.33.106`. The WiFi must allow devices
to communicate with each other; disable client isolation (AP isolation).

#### 2.3.2 Wired USB Connection

Use a **Type-C to Type-C data cable** to connect Pico's Type-C port to the
**Type-C port on the robot's back**. Keep Developer Mode and USB debugging
enabled on Pico.

Run the following in an SSH terminal on the **robot Orin**, not on the local
development computer. If Orin does not have `adb`, first run
`sudo apt install android-tools-adb`, then check the device:

```bash
adb devices
```

Confirm that Pico is listed as `device`. If it is `unauthorized`, approve USB
debugging in the headset. Continue with the shared steps below; Section 7 sets
up USB port forwarding immediately before starting the service.

## 3. Download the Project and Load the Environment

Clone this project on the robot Orin:

```bash
cd ~
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

Load the ROS 2 environment and the `drdds` message interface on the robot:

```bash
source /opt/ros/<ros-distro>/setup.bash
source <drdds-workspace>/install/setup.bash
```

The ROS 2 package name provided by `deep-robotics-msg` is `drdds`. If the
robot environment provides it as a system installation, use the setup file
specified for that delivery environment. If it was built from source, load
the workspace's `install/setup.bash`. If the robot environment cannot locate
the package, clone
[DeepRoboticsLab/deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)
on the robot Orin, follow that repository's instructions to build it, and load
the generated `install/setup.bash`.

Enter the project root:

```bash
cd ~/deep-robotics-teleoperate
```

## 4. Check the DR02 Pro Model Submodule

`deep_robotics_model` is pinned as a Git submodule at a version verified with
this project. When the project is cloned with `--recurse-submodules` as shown
in Section 3, the model is downloaded automatically into the project root. The
default IK model path is:

```text
deep-robotics-teleoperate/deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

If the project was previously downloaded with a regular `git clone`, or if the
submodule directory is empty, initialize it from the project root:

```bash
git submodule update --init --recursive
```

Confirm that the default URDF exists:

```bash
ls deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

Normal use does not require cloning the model repository separately or setting
an environment variable. If a verified `deep_robotics_model` checkout is
installed elsewhere, set the environment variable in the same terminal that
starts `state_machine` so that it points to the actual URDF file:

```bash
export DR02_PRO_TELEOP_URDF=/absolute/path/to/deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

When this environment variable is set, the program uses the specified URDF in
preference to the default path under the project root.

After initialization, the main project entries relevant to this workflow are:

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

## 5. Build for the Real Robot

Use a terminal without an active Conda environment and load the ROS 2 and
message-interface environments described above. If needed, run
`conda deactivate` until Conda is inactive before loading them. Run the
following commands from the project root:

```bash
colcon build --packages-up-to dr02_tele \
  --cmake-args -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

If a previous CMake configuration selected Conda libraries, add
`--cmake-clean-cache` before `--cmake-args` in the command above. Mixing Conda
and system OpenSSL libraries can cause undefined symbols at link time. The
unused `BUILD_PLATFORM` and `BUILD_SIM` warnings from `deep_robotics_common`
are harmless; these options are consumed by `dr02_tele`.

> [!WARNING]
>
> A real-robot build must use `BUILD_PLATFORM=arm` and keep `BUILD_SIM=OFF`.
> `BUILD_SIM=ON` disables the real-robot runtime safety monitor and must never
> be used on a physical robot.

## 6. Enter Upper-Body Joint-Control Developer Mode

Before starting teleoperation, use the robot gamepad to select and enter
**Upper-Body SDK Mode**. Clear the space in front of the chest, around the
head, and on both sides of the body before entering this mode.

The state machine publishes complete 31-DoF joint commands. Pico controls the
waist and both arms, while leg motion is handled by the robot's internal
policy. Do not switch Developer Modes while the robot is moving.

Before using the Zhixing grippers, start `zx_gripper_sdk` on RK.

## 7. Start Robotics Service

After compilation, open Terminal 1 and log in to the robot Orin:

```bash
ssh user@10.21.33.106
```

**Wired USB only:** confirm that `adb devices` lists Pico as `device`, then
set up port forwarding on Orin. Repeat this command after reconnecting the
cable or restarting either device:

```bash
adb reverse tcp:63901 tcp:63901
```

**For both modes**, start the service:

```bash
cd /opt/apps/roboticsservice/
./runService.sh
```

Keep this terminal and Robotics Service running. In USB mode, also keep the
USB data cable connected.

## 8. Configure Pico

Start Robotics Service on the robot Orin as described in Section 7, then
follow the steps for your connection mode.

### 8.1 Wireless WiFi

1. Open **Settings** → **WiFi** and connect Pico to the same WiFi as the robot
   Orin.
2. Open **Library** → **Unknown Sources** → **XRoboToolkit**.
3. Select **Enter**.
4. Set the receiver address to `<Orin-WiFi-IP>` from Section 2.3.1 and connect.
5. After connecting, enable **Controller** and **Send**. Enable **Head** only
   when waist control is needed; otherwise leave it unchecked. Keep the headset awake.

### 8.2 Wired USB

Keep the Type-C data cable connected between Pico and the robot's rear Type-C
port. Confirm that `adb reverse tcp:63901 tcp:63901` has been run on Orin as
described in Section 7.

1. Open **Library** → **Unknown Sources** → **XRoboToolkit**.
2. Select **Reconnect** directly. No receiver address is needed; confirm that
   **working** appears in green.
3. Enable **Controller** and **Send**. Enable **Head** only when waist control
   is needed; otherwise leave it unchecked. Keep the headset awake.

If you are using the `--record` option, it is recommended to disable **Head**.
Keep **Controller** and **Send** enabled.

## 9. Start the State Machine

You can reuse the terminal that completed the build from the project root in
Section 5. That terminal is already connected to the robot Orin and has the
required environment loaded, so run:

```bash
ros2 run dr02_tele state_machine
```

This default command enables only the arms and waist and creates no gripper
topics. To enable the Zhixing grippers, run:

```bash
ros2 run dr02_tele state_machine --ee zhixing
```

If the build terminal has been closed, or if you prefer to run the state
machine in a separate terminal, open Terminal 2, connect to `10.21.33.106`,
reload the environment, and start it:

```bash
ssh user@10.21.33.106
source /opt/ros/<ros-distro>/setup.bash
source <drdds-workspace>/install/setup.bash
cd ~/deep-robotics-teleoperate
source install/setup.bash
ros2 run dr02_tele state_machine
```

Append `--ee zhixing`, `--record`, or both to the separate-terminal command as
needed.

The state machine first waits for `/JOINTS_DATA`. When the control prompt
appears:

1. Press `z` or L1 on the robot gamepad to enter `ZeroPos`, then wait for the
   three-second transition to complete.
2. Press `s` or L2 on the robot gamepad to enter `Teleoperation`.
3. Wait for `PXREA server connected`, `device found`, and the first Pico data
   message to appear in the terminal.
4. Hold the trigger on one Pico controller (it may be fully depressed), then
   move that controller only a small amount to begin validation. After
   confirming that side works correctly, gradually validate the other
   controller and headset control.

For state transitions, Pico anchoring behavior, and diagnostic information,
see [State Machine and Pico Control](STATE_MACHINE.md).

## 10. Normal Shutdown and Emergency Handling

For a normal shutdown:

1. Release both triggers.
2. Press `d` or R2 on the robot gamepad to enter `Down`, then wait for the
   three-second transition to complete.
3. Press Ctrl+C in the `state_machine` terminal and confirm that the program
   exits.
4. Stop Robotics Service.
5. Exit the robot Developer Mode last.

If an abnormal condition occurs, use the red stop button on the robot gamepad
or the physical emergency stop first. Keyboard `e` and R1 on the robot gamepad
only enter the software `Emergency` pose hold and cannot replace a hardware
emergency stop.

## 11. Troubleshooting

| Symptom | Check first |
| --- | --- |
| SSH connection fails | Network between the development computer and robot, reachability of `10.21.33.106`, and SSH credentials |
| Continues to display `wait data update` | `/JOINTS_DATA`, the ROS 2 environment, `drdds`, and DDS configuration |
| PXREA initialization or loading fails | Whether Robotics Service is running and whether ARM dependencies are in use |
| URDF cannot be found | `deep_robotics_model/` in the project root or the `DR02_PRO_TELEOP_URDF` path |
| No Pico data over WiFi | Whether Pico and Orin share WiFi, the receiver address is `<Orin-WiFi-IP>`, transmission switches are enabled, and the headset is awake |
| No Pico data over USB | Cable connected to the robot's rear Type-C port, Pico listed as `device` by `adb devices` on Orin, port 63901 in `adb reverse --list`, Robotics Service running, green **working** status after **Reconnect**, and transmission switches enabled |
| Data arrives but the arms do not move | `Teleoperation` state, trigger/stale-recovery state, finite joint feedback, and initial arm positions within the loaded URDF limits |
| Repeated `reject IK candidate` | Check the logged residuals against the position/orientation thresholds; see [IK diagnostics](STATE_MACHINE.md#9-runtime-diagnostics-and-troubleshooting) |

> [!WARNING]
>
> - Run only one `/JOINTS_CMD` publisher at a time.
> - The first motion must be small and validated one side and one step at a
>   time.
> - Stop the SDK before exiting or switching Developer Mode.
> - The software safety monitor, Pico stale-data hold, and `Emergency` cannot
>   replace a hardware emergency stop.
