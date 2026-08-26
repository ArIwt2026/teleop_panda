# Franka Teach Tools

`franka_teach_tools` is a small ROS 2 utility package for teaching named points and ordered
point sequences from live joint states.

## Teaching workflow

1. Launch the robot in state-only mode so joint states keep updating and no motion controller
   fights hand-guiding.
2. Put the robot into hand-guiding on the robot itself.
3. Run the CLI and capture:
   - a single named point
   - or a sequence made of discrete named points

The tool does not record a continuous trajectory. It saves only the current joint state when you
trigger a capture.

## Recommended bringup

For a real robot, start the MoveIt stack in teaching mode. This keeps the state broadcasters and
RViz running while loading the arm trajectory controller inactive, so it cannot fight hand-guiding
or attempt to enter external control while the robot is user-stopped.

Example:

```bash
ros2 launch franka_moveit_config moveit.launch.py \
  robot_ip:=<fci-ip> start_cameras:=false teaching_mode:=true
```

Teaching mode intentionally does not reactivate the arm controller when the robot leaves
`USER_STOPPED` or `GUIDING`. Restart the launch without `teaching_mode:=true` when execution is
required.

## Commands

Capture a single point:

```bash
ros2 run franka_teach_tools teach_cli capture-point home
```

Capture a sequence of discrete points:

```bash
ros2 run franka_teach_tools teach_cli capture-sequence pick_demo
```

Require white-light / guiding mode if `robot_state` is available:

```bash
ros2 run franka_teach_tools teach_cli capture-sequence pick_demo \
  --robot-state-topic /panda/robot_state \
  --require-guiding
```

List saved data:

```bash
ros2 run franka_teach_tools teach_cli list-points
ros2 run franka_teach_tools teach_cli list-sequences
```

## Storage layout

By default, the tool stores files in `<workspace>/teach_data` when it can detect the colcon
workspace. Otherwise it falls back to `~/.ros/franka_teach_tools`.

Files are stored as YAML:

```text
teach_data/
  points/
    home.yaml
    pick_ready.yaml
  sequences/
    pick_demo.yaml
```

Sequences reference named point files so later automation code can load points or whole sequences
by name.

## Execution workflow

Start your normal execution stack first. The executor launch does not start MoveIt, RViz, or a
controller manager. It only starts the taught-point executor node and attaches to the already
running stack.

For example, you can first start your usual MoveIt/controller setup, then start the executor:

```bash
ros2 launch franka_teach_executor executor.launch.py
```

Then call the executor from the CLI:

```bash
ros2 run franka_teach_tools executor_cli play-point home
ros2 run franka_teach_tools executor_cli play-relative rel_pick_slot_1 --pipeline ompl --execute
ros2 run franka_teach_tools executor_cli play-sequence pick_demo
ros2 run franka_teach_tools executor_cli pause
ros2 run franka_teach_tools executor_cli continue
ros2 run franka_teach_tools executor_cli stop
ros2 run franka_teach_tools executor_cli recover
ros2 run franka_teach_tools executor_cli status
```

The executor replays taught points through MoveIt and the existing trajectory controller. It is
meant for execution mode, not white-light teaching mode, and it should not open a second control
stack. For relative points, `--pipeline ompl` plans the approach to the retracted pose with OMPL;
the final descent remains a Pilz `LIN` move.
