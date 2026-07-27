# DR02 Pro Pico Teleoperation Guide

**English** | [简体中文](README_CN.md) ·
[Repository overview](../../README.md)

`dr02_tele` is the DR02 Pro Pico upper-body teleoperation package. Its only
runtime program is `state_machine`, which connects three data paths:

- XRoboToolkit/PXREA supplies Pico headset and controller data.
- ROS 2/DDS supplies joint feedback from the real robot or simulator.
- The state machine publishes position-PD targets through `/JOINTS_CMD`.

## Runtime Components

| Component | Runs on | Purpose |
| --- | --- | --- |
| XRoboToolkit | Pico headset | Sends headset pose, controller poses, buttons, and trigger data. |
| Robotics Service | Teleoperation host | Receives Pico data and provides the local PXREA service endpoint. |
| `state_machine` | Same host as Robotics Service | Receives PXREA and ROS 2 data, calculates targets, and publishes commands. |
| DR02 Pro or simulator | ROS 2/DDS network | Publishes feedback and receives joint commands. |

Simulation teleoperation runs on an x86 simulation host. Real-robot
teleoperation runs on the robot Orin at `10.21.33.106`. The Pico receiver
address always points to the host running Robotics Service and `state_machine`.

## Choose a Workflow

| Goal | Document | Build configuration |
| --- | --- | --- |
| Simulation teleoperation | [Simulation Teleoperation](docs/SIMULATION.md) | x86 host, `BUILD_PLATFORM=x86`, `BUILD_SIM=ON` |
| Real-robot teleoperation | [Real-Robot Teleoperation](docs/REAL_ROBOT.md) | Robot Orin, `BUILD_PLATFORM=arm`, `BUILD_SIM=OFF` |

After completing the selected workflow, use
[State Machine and Pico Control](docs/STATE_MACHINE.md) as the reference for
state transitions, Pico behavior, protection mechanisms, diagnostics, and
troubleshooting.

## Directory Layout

- `manipulation/`: PXREA reception, Pico-to-robot mapping, filtering, and
  dual-arm IK
- `state_machine/`: ROS 2 interfaces, input handling, runtime safety monitor,
  and the `state_machine` executable
- `docs/`: complete real-robot and simulation workflows plus the state-machine
  reference

> [!WARNING]
>
> - Run only one `/JOINTS_CMD` publisher at a time.
> - `Emergency` is a software pose hold, not a hardware emergency stop.
> - Validate the complete workflow in simulation before controlling a real
>   robot for the first time. Real-robot validation must use a clear workspace
>   and begin with small motions.
