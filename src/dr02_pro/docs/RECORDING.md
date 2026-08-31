# DR02 Pro Three-Camera Data Collection

**English** | [简体中文](RECORDING_CN.md) ·
[Back to the DR02 Pro Pico Teleoperation Guide](../README.md)

This document describes the three-camera RGB data-collection feature integrated
into `state_machine`. It does not start a separate process or add ROS 2/DDS
recording topics. Disabling **Head** on the PICO headset is recommended when
data collection is enabled.

> [!IMPORTANT]
>
> Before enabling data collection, complete the full real-robot teleoperation
> workflow in the [real-robot teleoperation guide](REAL_ROBOT.md). On the first
> run, start `state_machine` without `--record` and verify state transitions,
> PICO dual-arm teleoperation. If gripper data is required, also verify it with
> `--ee zhixing`. Then configure all three RealSense cameras and add `--record`
> to the same command. Data collection extends the real-robot workflow; it is not a
> separate startup procedure.

## Configuration Before Startup

First confirm that the RealSense SDK (librealsense) is installed on the robot
Orin. If it is not installed, follow the
[RealSense Camera Guide in deep-robotics-sdk2](https://github.com/DeepRoboticsLab/deep-robotics-sdk2/blob/main/src/dr02_pro/docs/CAMERA.md).

List the connected RealSense devices and their serial numbers:

```bash
rs-enumerate-devices -s
```

If you cannot determine which D405 is mounted on the left or right wrist,
enable X11 forwarding when opening the SSH connection:

```bash
ssh -Y user@10.21.33.106
realsense-viewer
```

Inspect each camera image in `realsense-viewer` to identify its mounting
position. Close `realsense-viewer` afterward because it cannot share a camera
with the recorder.

Enter the confirmed serial numbers in left-wrist, right-wrist, and head order
in `kCameraSerials` within
`src/dr02_pro/state_machine/src/recording_manager.cpp`, then rebuild by
following the [real-robot teleoperation guide](REAL_ROBOT.md).

## Startup

Enable data collection with the default directory:

```bash
ros2 run dr02_tele state_machine --record
```

The default dataset directory is `record_data/` under the directory from which
the command is launched. The program converts it to an absolute path and
prints it at startup.

To select another directory:

```bash
ros2 run dr02_tele state_machine --record \
  --record-dir /absolute/path/to/record_data
```

`--record-dir` must be used together with `--record`. Without `--record`,
the program does not initialize the cameras, synchronizer, or HDF5 writer
thread.

`--record` alone stores 14 arm dimensions. To control Zhixing grippers and
store the 16-dimensional layout, use:

```bash
ros2 run dr02_tele state_machine --ee zhixing --record
```

## Recording Controls

1. Follow the normal state-machine sequence and enter `ZeroPos`.
2. Press `s` or L2 on the robot gamepad to enter `Teleoperation`.
3. Confirm that all three cameras are reported online.
4. Press the primary button on the left PICO controller to start an episode.
5. Press the same button again after completing the motion to finish and save
   the episode.
6. While not recording, press the secondary button on the left PICO controller
   to delete the most recent `episode_N.hdf5`.

Recording buttons are active only in `Teleoperation`. A button already held
before entering teleoperation does not trigger an operation; release it and
press it again.

Neither trigger nor grip controls the recording state:

- Releasing a trigger holds the corresponding arm at its latest safe target
  while images, states, and actions continue to be written.
- With `--ee zhixing`, releasing a grip only changes the gripper target.
- An episode can be finished normally only by pressing the left primary button
  again while in teleoperation.

If the state machine enters `Down` or `Emergency`, activates a safety
protection, or receives `Ctrl+C` during recording, the incomplete episode is
closed and deleted instead of producing a final HDF5 file.

With `--ee zhixing --record`, a fresh valid gripper feedback sample is also
required to start. A feedback gap longer than `200 ms` aborts and deletes the
current episode while arm teleoperation continues.

## Camera Configuration

The default camera positions and capture settings are listed below. Fill serial
numbers from the deployed devices; real device IDs are not hard-coded in this
document.

| Camera | Serial number | Configuration |
| --- | --- | --- |
| Left-wrist D405 | `<left-wrist-d405-serial>` | RGB8, 640 × 480, 30 FPS |
| Right-wrist D405 | `<right-wrist-d405-serial>` | RGB8, 640 × 480, 30 FPS |
| Head D435 | `<head-d435-serial>` | RGB8, 640 × 480, 30 FPS |

Recording can start only when all three cameras are online; the program does
not substitute black frames. The maximum synchronization skew is 20 ms. When
the skew exceeds that limit, the earliest frame is discarded and
synchronization is retried.

A camera is considered offline after two seconds without a valid frame:

- When idle, the recorder rejects new episodes while teleoperation continues.
- During recording, the current incomplete episode is deleted automatically.
- The camera is retried every two seconds, and recording becomes available
  again after all three cameras recover.

If the build does not detect the RealSense SDK, teleoperation remains
available, but `--record` reports that data collection is unavailable.

## Dataset Directory

```text
record_data/
├── episode_0.hdf5
├── episode_1.hdf5
└── ...
```

An episode is first written to:

```text
episode_N.hdf5.tmp
```

Only after a normal finish and successful flush/close is the file atomically
renamed to `episode_N.hdf5` in the same directory. Zero-frame episodes,
camera loss, write failures, or leaving teleoperation delete the temporary
file. Stale temporary files from an abnormal exit are also removed at startup.

After the most recent episode is deleted, its number is reused by the next
recording. Deletion acts directly on the filesystem and cannot be undone.

## HDF5 Format

Each row contains one synchronized set of three camera images together with
the nearest robot state and action. The target storage rate is approximately
30 Hz.

Without an end effector the 14-dimensional order is:

```text
left arm 7, right arm 7
```

With `--ee zhixing`, the 16-dimensional order is:

```text
left arm 7, right arm 7, left gripper 1, right gripper 1
```

The waist is not included in the current dataset contract. Arm `qpos` values
are measured joint positions, and arm `action` values are the filtered
position targets actually published. Grippers follow the CR1 convention:

```text
qpos   = 1 - clamp(hand_feedback / 1000, 0, 1)
action = 1 - clamp(grip, 0, 1)
```

Gripper `qpos` always comes from valid measured feedback and is never filled
from `action`.

```text
episode_N.hdf5
├── attributes
│   ├── fps = 30
│   ├── state_fps = 30
│   ├── state_dimension = D
│   ├── state_layout
│   ├── end_effector = "none" or "zhixing"
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
    ├── control_mode                float64 [N], always 1
    ├── gate                        float64 [N], always 0
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

`timestamps` is the minimum of the three camera timestamps.
`camera_sync_skew_ms` is the difference between the maximum and minimum
camera timestamps in the same row, and
`state_time_offset_ms = state_timestamps - timestamps`. The two root
attributes prefixed with `max_*` store the maximum absolute offsets across the
episode.

RGB images are encoded as JPEG at quality 95. Each image dataset uses a fixed
row length of 500000 bytes, while `*_len` stores the actual JPEG length.
HDF5 `deflate(1)` compresses the zero padding at the end of each row; it does
not alter JPEG quality or the existing schema. Readers must slice each row to
`*_len` before decoding the JPEG.

## Pre-Collection Checklist

Before collecting real-robot data, confirm that:

1. The three camera serial numbers match the deployed configuration and
   all cameras are reported online.
2. `/JOINTS_DATA` continues to update; 16-dimensional collection additionally
   requires continuously updated feedback for both sides on
   `/gripper/position` from the RK SDK.
3. Pressing the left primary button prints `episode N started`.
4. The recorder remains active after trigger release; grip release also keeps
   recording active in Zhixing mode.
5. Pressing the left primary button again leaves only the final `.hdf5` file
   and no `.tmp` file.
6. Every dataset has a first dimension equal to `episode_len`, and sample
   JPEG rows from all three cameras can be decoded.
