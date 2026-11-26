# DR02 Pro Pico Teleoperation

**English** | [简体中文](README_CN.md) ·
[Repository overview](../../README.md)

`dr02_tele` provides the `state_machine` executable for DR02 Pro upper-body
teleoperation. PICO always controls the waist and both arms. End-effector
control is disabled by default and can be enabled explicitly at runtime.

## Choose a Workflow

| Target | Platform | Build settings | Complete guide |
| --- | --- | --- | --- |
| MuJoCo simulation | x86_64 | `BUILD_PLATFORM=x86`, `BUILD_SIM=ON` | [Simulation](docs/SIMULATION.md) |
| DR02 Pro robot | ARM64 Orin | `BUILD_PLATFORM=arm`, `BUILD_SIM=OFF` | [Real robot](docs/REAL_ROBOT.md) |

On the real robot, connect Pico over [WiFi](docs/REAL_ROBOT.md#231-wireless-wifi-connection)
or [USB](docs/REAL_ROBOT.md#232-wired-usb-connection). Both modes share the build
and teleoperation workflow.

The PICO headset, Robotics Service, `state_machine`, and robot or simulator
are separate runtime components. Follow the selected guide for installation,
networking, build, and startup order.

## Runtime Options

```bash
ros2 run dr02_tele state_machine [--log] [--record] [--record-dir PATH] [--ee zhixing]
```

| Option | Purpose |
| --- | --- |
| `--log` | Print PICO and control diagnostics once per second. |
| `--record` | Enable integrated three-camera HDF5 episode recording. |
| `--record-dir PATH` | Select the recording directory; requires `--record`. |
| `--ee zhixing` | Enable the Zhixing dual gripper. Without this option no gripper topics are created. |

Common combinations:

| Command suffix | Control and recording layout |
| --- | --- |
| none | Arms and waist only; no recording |
| `--record` | Arms and waist; 14-dimensional arm data |
| `--ee zhixing` | Arms, waist, and Zhixing grippers; no recording |
| `--ee zhixing --record` | Arms, waist, and Zhixing grippers; 16-dimensional data |

Recording requires RealSense support and all three configured cameras. See the
[recording guide](docs/RECORDING.md) for camera mapping, controls, and file
format.

## Controls

| Input | Action |
| --- | --- |
| `z` / L1 | Enter `ZeroPos` from `Emergency` or `Down`. |
| `s` / L2 | Enter `Teleoperation` from `ZeroPos`. |
| `d` / R2 | Enter `Down` from `Emergency` or `Teleoperation`. |
| `e` / R1 | Enter the software `Emergency` pose hold. |
| PICO trigger | Anchor and control the corresponding arm. |
| PICO grip | Control the corresponding gripper when `--ee zhixing` is enabled. |
| PICO left primary / secondary | With `--record`, start or finish an episode / delete the most recent completed episode. |
| PICO right primary / secondary | No application function is bound. |
| PICO joystick axes / clicks | Not parsed or bound. |

## ROS 2 Interfaces

| Topic | Direction | Purpose |
| --- | --- | --- |
| `/JOINTS_DATA` | Subscribe | 31-DoF joint feedback |
| `/JOINTS_CMD` | Publish | 31-DoF joint command |
| `/gripper/position` | Subscribe | Feedback `[side, position]` from the new Zhixing SDK on RK; exists only with `--ee zhixing` |
| `/gripper/position_cmd` | Publish | Dual-gripper command `[2, left, right]` to the new Zhixing SDK on RK; exists only with `--ee zhixing` |
| `/GAMEPAD_KEY` | Subscribe | Robot gamepad state commands |

Only one joint-command publisher may run on the same robot or simulator.
Before using the Zhixing grippers, start `zx_gripper_sdk` on RK.

## Safety Behavior

- The real-robot build monitors joint feedback and motor temperature.
- Invalid joint values or a feedback gap longer than `100 ms` request
  `Emergency`; unchanged values remain valid when complete frames continue to
  arrive.
- Stale PICO input stops target updates and requires a safe recovery sequence.
- Teleoperation initialization rejects non-finite joint data or arm positions
  outside the loaded URDF limits, allowing only `1e-4 rad` rounding tolerance.
- `Emergency` holds a pose in software. It does not replace the robot's
  physical emergency stop or hardware protection.

Always validate in simulation before first use on hardware. For state details,
PICO behavior, and troubleshooting, see
[State Machine and PICO Control](docs/STATE_MACHINE.md).

## Package Layout

```text
src/
├── end_effector/         Optional end-effector interface, factory, and implementations
└── dr02_pro/
    ├── manipulation/     PICO reception, input shaping, and dual-arm IK
    ├── state_machine/    ROS 2 interfaces, state control, safety, and recording
    └── docs/             Simulation, real-robot, control, and recording guides
```
