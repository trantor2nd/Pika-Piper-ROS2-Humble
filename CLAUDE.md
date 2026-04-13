# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 Humble-based teleoperation and data collection system for the Pika robotic arm (6-DOF + gripper). The pipeline supports keyboard-driven control, synchronized multi-camera recording, trajectory replay, and dataset conversion to LeRobot v3 format for imitation learning.

## Environment Setup

All scripts assume a `py310` conda environment with ROS2 Humble sourced:

```bash
source ~/miniforge3/etc/profile.d/conda.sh && conda activate py310
source /opt/ros/humble/setup.bash
source /home/data/Project/piper_ros/install/setup.bash

# Required Python path exports (already set in shell scripts)
export PYTHON_EXECUTABLE=/home/hsb/miniforge3/envs/py310/bin/python
export PYTHONPATH=/home/hsb/miniforge3/envs/py310/lib/python3.10/site-packages:$PYTHONPATH
```

## Running

**Keyboard teleoperation + data recording** (one command, handles CAN bus, gripper, cameras):
```bash
bash start_piper_keyboard.sh
```

**Trajectory replay:**
```bash
RATE=10 WARMUP=3 bash start_piper_replay.sh episode_00000 --loop
```

**Dataset conversion to LeRobot v3:**
```bash
python convert2lerobot.py --raw-root /path/to/raw --output-path /path/to/output --fps 10 --overwrite
```

**Camera/gripper hardware test:**
```bash
python test_camera.py
```

**VR teleoperation** (requires two terminals, see `teleoperate/`):
- Terminal 1: `bash teleoperate/terminal1.sh` (CAN + sensor/gripper launch)
- Terminal 2: `bash teleoperate/terminal2.sh` (ROS2 teleop launch)
- Calibration: `bash teleoperate/calibrate.sh`

## Architecture

All Python scripts are standalone ROS2 nodes (no ROS2 package build needed — run directly with `python3`).

### Node Communication (ROS2 Topics)

```
keyboard_piper_control.py          data_recorder.py
        |                                |
        |--/joint_ctrl_single (pub)-->   |
        |<--/joint_states_single (sub)---|  (from piper_single_ctrl)
        |--/gripper_state (pub)--------->|  (sub)
        |--/record_cmd (pub)------------>|  (sub: "start"/"stop"/"exit")
        |--/enable_flag (pub)-->         |
```

- **keyboard_piper_control.py** — Keyboard teleop node. Publishes joint commands and gripper state, controls recording via `/record_cmd` topic. Uses `SingleThreadedExecutor` + `select()` for non-blocking keyboard input.
- **data_recorder.py** — Subscribes to joint state and gripper feedback; captures synchronized frames from Fisheye + RealSense cameras in a background thread at 10Hz. Saves episodes as numbered JPEGs + `.log` state files.
- **sample_deploy.py** — Trajectory replay node. Loads `.log` files from a recorded episode, sends joint+gripper commands at configurable rate. Linear interpolation warmup to reach first waypoint.
- **convert2lerobot.py** — Offline converter from raw dataset format to LeRobot v3 (Parquet + MP4). Infers state/action dimensions automatically.
- **pika_rs_revise.py** — Commented-out RealSense camera driver revision (not active).

### Raw Dataset Format

```
piper_dataset_raw/
  <task_name>/
    episode_00000/
      img/fisheye_rgb/00001.jpg, 00002.jpg, ...
      img/realsense_rgb/00001.jpg, 00002.jpg, ...
      state/00001.log, 00002.log, ...  (space-separated: j0 j1 j2 j3 j4 j5 gripper)
```

## Key Constants (shared across nodes)

```python
JOINT_LIMITS = [(-3.14, 3.14), (0.0, 2.0), (-2.0, 0.0), (-1.5, 1.8), (-1.3, 1.57), (-3.14, 3.14)]
HOME_POS = [0.0, -0.035, 0.0, 0.0, 0.35, 0.0]
GRIPPER_RANGE = [0.0, 90.0]  # mm, closed to open
```

## Hardware Dependencies

- CAN bus interface (`can0`) — activated via `/home/data/Project/piper_ros/can_activate.sh can0 1000000`
- Gripper serial port: `/dev/ttyUSB0` (keyboard mode) or `/dev/ttyUSB1` (replay mode)
- Fisheye camera: device index 6
- RealSense D405: serial `230322275684` (falls back to any available device)
- Shell scripts run `sudo chmod 666 /dev/ttyUSB*` for port access

## Key Libraries

- `rclpy` / `sensor_msgs` / `std_msgs` — ROS2 Python
- `pika.gripper.Gripper` — Gripper control via serial
- `pika.camera.fisheye.FisheyeCamera` / `pika.camera.realsense.RealSenseCamera` — Camera drivers
- `lerobot.datasets.lerobot_dataset.LeRobotDataset` — Dataset conversion target
- `piper_msgs.srv.Enable` — Optional robot enable service (from piper_ros workspace)

## Keyboard Controls (keyboard_piper_control.py)

| Key | Action |
|-----|--------|
| a/d, w/s, u/j, r/f, t/g, q/e | Joint 0-5 +/- |
| h / k | Gripper open / close (10mm steps) |
| o / p | Start / stop recording |
| z | Return to HOME position |
| Space | Safe shutdown and exit |
