# DR02 Pro Real-Robot Teleoperation

**English** | [简体中文](REAL_ROBOT_CN.md) ·
[Back to the DR02 Pro Pico Teleoperation Guide](../README.md)

Real-robot teleoperation is supported only on the robot Orin at
`10.21.33.106`. Robotics Service and `state_machine` both run on that device.

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

### 2.3 Connect the Robot Orin to the Required WiFi

Continue in the current SSH terminal. Do not disconnect the Ethernet cable
between the local development computer and the robot or close the current SSH
session while configuring WiFi. First confirm through the wired connection
that WiFi has obtained an IP address.

1. List the Orin network devices and identify the wireless interface whose
   `TYPE` is `wifi`:

   ```bash
   nmcli device status
   ```

   The commands below use `<wifi-interface>` for this name, for example
   `wlan0` or `wlP1p1s0`.

2. Enable WiFi and scan for networks visible to that interface:

   ```bash
   sudo nmcli radio wifi on
   nmcli device wifi list ifname <wifi-interface>
   ```

3. Connect to the target WiFi. `--ask` prompts for the password in the terminal
   so that it does not appear directly in the command or shell history:

   ```bash
   sudo nmcli --ask device wifi connect "<wifi-ssid>" \
     ifname <wifi-interface> name "pico-teleop-wifi"
   ```

   For a hidden SSID, use:

   ```bash
   sudo nmcli --ask device wifi connect "<wifi-ssid>" \
     ifname <wifi-interface> name "pico-teleop-wifi" hidden yes
   ```

4. Check the connection and query the IPv4 address assigned to the Orin on
   this WiFi:

   ```bash
   nmcli connection show --active
   nmcli -g IP4.ADDRESS device show <wifi-interface>
   ```

   The output is normally `<Orin-WiFi-IP>/<prefix-length>`, for example
   `192.168.1.20/24`. `<Orin-WiFi-IP>` is the receiver address to enter later
   in the Pico XRoboToolkit application. `10.21.33.106` is the wired address
   currently used to access the Orin. Do not treat it as the WiFi IP unless the
   on-site network has explicitly assigned the same address.

5. Connect the Pico to the same WiFi. The WiFi must allow devices on the same
   LAN to communicate and must not enable AP isolation (client isolation).

NetworkManager saves this connection as `pico-teleop-wifi`. To reconnect it
manually later, run:

```bash
sudo nmcli connection up "pico-teleop-wifi"
```

If `nmcli` is unavailable, no wireless interface appears, or the device state
is `unmanaged`, do not directly modify the robot network configuration. Follow
the network procedure supplied with the robot or contact technical support.

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

Run the following commands from the project root:

```bash
colcon build --packages-up-to dr02_tele \
  --cmake-args -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF
source install/setup.bash
```

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

After compilation, open a new terminal (Terminal 1), connect to
`10.21.33.106`, and run the following commands on the robot Orin:

```bash
ssh user@10.21.33.106
cd /opt/apps/roboticsservice/
./runService.sh
```

Keep this terminal and Robotics Service running.

## 8. Configure Pico

On the Pico:

1. Open **Settings** → **WiFi** and connect the Pico to the same WiFi as the
   robot Orin.
2. Open **Library** → **Unknown Sources** → **XRoboToolkit**.
3. Select **Enter**.
4. Set the receiver address to the `<Orin-WiFi-IP>` queried earlier. This is
   the address on the current WiFi of the robot Orin running Robotics Service
   and `state_machine`.
5. After the connection succeeds, enable **Controller** and **Send**. Also
   enable **Head** for normal teleoperation. If `state_machine` is started with
   `--record`, keeping **Head** disabled is recommended because the current
   dataset contract does not record waist-joint data.
6. Keep the headset awake so that sleep does not interrupt data transmission.

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
| No Pico data | Whether Pico and Orin are on the same WiFi, the receiver address is `<Orin-WiFi-IP>`, all three transmission switches are enabled, and the headset is awake |
| Data arrives but the arms do not move | Whether the state is `Teleoperation`, the trigger reaches its threshold, and the trigger must first be released |

> [!WARNING]
>
> - Run only one `/JOINTS_CMD` publisher at a time.
> - The first motion must be small and validated one side and one step at a
>   time.
> - Stop the SDK before exiting or switching Developer Mode.
> - The software safety monitor, Pico stale-data hold, and `Emergency` cannot
>   replace a hardware emergency stop.
