# DR02 Pro Simulation Teleoperation

**English** | [简体中文](SIMULATION_CN.md) ·
[Back to the DR02 Pro Pico Teleoperation Guide](../README.md)

This document provides the complete SDK-side workflow for DR02 Pro Pico
simulation teleoperation. The MuJoCo simulator is not included in this
repository. Follow
[deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation)
for its installation and environment setup.

> [!IMPORTANT]
>
> The SDK, Robotics Service, and simulator are separate processes. Running this
> simulation requires three terminal windows. They must use compatible ROS 2,
> `drdds`, and `ROS_DOMAIN_ID` environments. This program runs on Ubuntu 22.04
> by default.

## 1. Prepare Pico and Robotics Service

### 1.1 Install XRoboToolkit on Pico

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

### 1.2 Install Robotics Service on the Local Development Computer

Simulation runs by default on an Ubuntu 22.04 x86 (amd64) development
computer. Open
[XRoboToolkit PC Service Releases](https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases)
and download `XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb`. Enter the
directory containing the package and run:

```bash
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

After a successful installation, the `roboticsservice` directory is available
under `/opt/apps`:

```text
/opt/apps/roboticsservice/
```

Connect the Pico and the local development computer to the same WiFi, then set
the receiver address to this simulation host's IP address on that WiFi,
because Robotics Service and `state_machine` both run on the host.

## 2. Download the Projects

Simulation teleoperation requires these three repositories:

- [DeepRoboticsLab/deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)
- [DeepRoboticsLab/deep-robotics-simulation](https://github.com/DeepRoboticsLab/deep-robotics-simulation)
- [DeepRoboticsLab/deep-robotics-teleoperate](https://github.com/DeepRoboticsLab/deep-robotics-teleoperate)

On the development computer, create a common project directory and clone the
three repositories side by side into it:

```bash
cd ~/your_project
git clone https://github.com/DeepRoboticsLab/deep-robotics-msg.git
git clone https://github.com/DeepRoboticsLab/deep-robotics-simulation.git
git clone --recurse-submodules \
  https://github.com/DeepRoboticsLab/deep-robotics-teleoperate.git
```

The resulting directory layout is:

```text
your_project/
├── deep-robotics-msg/
├── deep-robotics-simulation/
└── deep-robotics-teleoperate/
    └── deep_robotics_model/  # Git submodule
```

Keep all three repositories at the same level. Do not clone one repository
inside another.

## 3. Check the DR02 Pro Model Submodule

`deep_robotics_model` is pinned as a Git submodule at a version verified with
the teleoperation project. When the project is cloned with
`--recurse-submodules` as shown in Section 2, the model is downloaded
automatically into the teleoperation project root. The default IK model path
is:

```text
deep-robotics-teleoperate/deep_robotics_model/DR02/urdf/pro/DR02-pro.urdf
```

If the teleoperation project was previously downloaded with a regular
`git clone`, or if the submodule directory is empty, run:

```bash
cd ~/your_project/deep-robotics-teleoperate
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

After initialization, the main entries in the teleoperation project relevant
to this workflow are:

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

## 4. Build for Simulation

### 4.1 Build and Load the Message Interfaces

First load the ROS 2 environment, then enter the
[deep-robotics-msg](https://github.com/DeepRoboticsLab/deep-robotics-msg)
repository downloaded in Section 2. Its local directory is named
`deep-robotics-msg`:

```bash
source /opt/ros/<ros-distro>/setup.bash
cd ~/your_project/deep-robotics-msg
```

Follow that repository's README to complete the build. After a successful
build, load the generated message interface environment in the same terminal:

```bash
source install/setup.bash
```

### 4.2 Build `dr02_tele`

Keep the ROS 2 and message interface environments loaded in the current
terminal, enter the teleoperation project, and build the simulation version:

```bash
cd ~/your_project/deep-robotics-teleoperate
colcon build --packages-up-to dr02_tele \
  --cmake-args -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON
source install/setup.bash
```

If you use a new terminal to build `dr02_tele`, reload the environments in
this order first:

```bash
source /opt/ros/<ros-distro>/setup.bash
source ~/your_project/deep-robotics-msg/install/setup.bash
```

`BUILD_SIM=ON` only disables the runtime `SafeController` monitor used on the
real robot. It does not start the simulator or change the ROS Topics or Pico
control logic.

> [!WARNING]
>
> This build is for simulation only and must not be used to control a real
> robot.

### 4.3 Configure `deep-robotics-simulation`

Open a new terminal and enter the simulation repository downloaded in Section
2:

```bash
cd ~/your_project/deep-robotics-simulation
```

Follow that repository's README to complete the build and environment setup.
Its configuration and startup procedure may change, so the repository's
current README takes precedence.

## 5. Start a Complete Simulation Session

The simulator and state machine must use the same `ROS_DOMAIN_ID`. This workflow
uses `10` as a concrete example, so `export ROS_DOMAIN_ID=10` must remain
consistent in every relevant terminal below. The value `10` has no special
meaning; if another value is required, change it in all relevant terminals.

Start the components in this order:

1. Start Robotics Service in the first terminal on the simulation host:

   ```bash
   cd /opt/apps/roboticsservice/
   ./runService.sh
   ```

2. On the Pico, first open **Settings** → **WiFi** and connect to the same WiFi
   as the local development computer. Then open **Library** → **Unknown
   Sources** → **XRoboToolkit** and select **Enter**. Set the receiver address
   to the simulation host's IP address on that WiFi. After the connection
   succeeds, enable **Head**, **Controller**, and **Send**, and keep the headset
   awake.

3. In the second terminal, enter the external simulation repository and start
   the DR02 Pro simulator. This is the current launch command; if the
   simulation repository changes, follow its documentation:

   ```bash
   cd ~/your_project/deep-robotics-simulation
   export ROS_DOMAIN_ID=10
   source /opt/ros/<ros-distro>/setup.bash
   source ~/your_project/deep-robotics-msg/install/setup.bash
   source .venv/bin/activate
   python3 run_sim.py dr02_pro
   ```

4. If needed, open another terminal and confirm that the simulator is
   continuously publishing joint feedback:

   ```bash
   export ROS_DOMAIN_ID=10
   source /opt/ros/<ros-distro>/setup.bash
   source ~/your_project/deep-robotics-msg/install/setup.bash
   ros2 topic hz /JOINTS_DATA
   ```

   After observing several samples, press Ctrl+C to stop the check.

5. Start the state machine in the third terminal:

   ```bash
   cd ~/your_project/deep-robotics-teleoperate
   export ROS_DOMAIN_ID=10
   source /opt/ros/<ros-distro>/setup.bash
   source ~/your_project/deep-robotics-msg/install/setup.bash
   source install/setup.bash
   ros2 run dr02_tele state_machine
   ```

6. Wait for the first `/JOINTS_DATA` frame and the control prompt. In the state
   machine terminal, press `z` to enter `ZeroPos`. After the three-second
   transition completes, press `s` to enter `Teleoperation`.
7. Wait for `PXREA server connected`, `device found`, and the first Pico data
   message to appear in the terminal. Hold the trigger on one Pico controller
   (it may be fully depressed), then move that controller only a small amount
   to begin validation.

The default command creates no gripper topics. To inspect the Zhixing command
path, start with `ros2 run dr02_tele state_machine --ee zhixing`. The current
simulator does not display gripper motion, so inspect the topic in another
terminal with the same ROS 2 environment and `ROS_DOMAIN_ID`:

```bash
ros2 topic echo /gripper/position_cmd
```

Entering `Teleoperation` does not publish an automatic open command. The first
message is sent only after fresh post-entry grip input. Its format is
`[2, left, right]`, where `2` selects both sides and the remaining positions
follow their grip values from `0` to `1000`. Without an RK SDK publishing
`/gripper/position`, 16-dimensional recording cannot start in simulation.

For state transitions, Pico control, and diagnostic output, see
[State Machine and Pico Control](STATE_MACHINE.md).

## 6. End the Simulation Session

1. Release both triggers.
2. In the state machine terminal, press `d` to enter `Down`, then wait for the
   three-second transition to complete.
3. Press Ctrl+C in the state machine terminal.
4. Stop the simulator and Robotics Service.

To enter the software `Emergency` pose hold, press `e` in the state machine
terminal.

## 7. Troubleshooting

| Symptom | Check first |
| --- | --- |
| `state_machine` continues waiting for data | Whether the simulator is continuously publishing `/JOINTS_DATA`, and whether all terminals use the same `ROS_DOMAIN_ID` |
| Simulator receives no commands | Check `/JOINTS_CMD`, state feedback, and the ROS 2/DDS environment; check `/gripper/position_cmd` only with `--ee zhixing` |
| Grip produces no visible motion | The current simulator does not display gripper motion; inspect `/gripper/position_cmd` instead |
| PXREA initialization fails | Whether `BUILD_PLATFORM=x86` was used and Robotics Service is running |
| `packets=0` | Pico receiver IP, the **Send** switch, and Robotics Service |
| `pico_age_ms > 200` | Whether the headset is asleep, the transmission switches, and the network between Pico and the host |
| IK initialization fails | DR02 Pro URDF path and model version |

> [!NOTE]
>
> Simulation can validate the teleoperation command path, but it cannot replace
> Developer Mode, workspace, and hardware emergency-stop checks on the real
> robot.
