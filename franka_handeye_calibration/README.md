# Franka Hand-Eye Calibration

`franka_handeye_calibration` coordinates event-driven hand-eye calibration runs on top of
taught Franka point sequences.

It uses:
- `franka_teach_executor` for point-to-point sequence playback
- ROS image topics for camera capture
- ChArUco pose estimation for board detection
- `FrankaRobotState.o_t_ee` as the primary robot pose source
- MoveIt's `/compute_fk` service to compute a validation FK pose from joint states

## One-command launch

```bash
ros2 launch franka_handeye_calibration handeye_calibration.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  job_name:=d455_handeye
```

This launch can start the MoveIt/control stack, the taught-sequence executor, the RealSense
camera stack, and the calibration coordinator in one place. By default it also starts the
job automatically and opens the OpenCV preview window so you can watch the ChArUco detection
and sample collection live. Pass `autostart:=false` if you want to start the job later from
the CLI. The camera side now uses the ROS `realsense2_camera` driver through the
`handeye_d455_front` camera suite by default.

## CLI

Run a job through the action server:

```bash
ros2 run franka_handeye_calibration handeye_cli run-job
```

Collect only:

```bash
ros2 run franka_handeye_calibration handeye_cli collect-job
```

Solve an existing run:

```bash
ros2 run franka_handeye_calibration handeye_cli solve-run /path/to/run_dir
```

Validate an existing run:

```bash
ros2 run franka_handeye_calibration handeye_cli validate-run /path/to/run_dir
```

## Temporary Sequence Debug Launch

If you only want to verify that a taught sequence advances on fake hardware, without a camera
or the real robot, use the temporary debug launch:

```bash
ros2 launch franka_handeye_calibration temp_sequence_debug.launch.py robot_ip:=dont-care
```

This uses a mock capture mode, skips the RealSense stack, and records synthetic samples only
so the event-driven sequence flow can be tested end to end.
