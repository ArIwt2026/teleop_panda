# Franka bringup controllers

## Panda admittance controller (ROS 2 Humble)

The namespaced camera launch starts Cartesian impedance first, then admittance.
On activation, admittance reads the current joint state from the Franka hardware
interfaces and moves to the configured home pose with a smooth, capped-speed
trajectory. It holds there until the Panda double-press button requests
admittance. A second double press requests the same smooth return to home.

```bash
ros2 launch franka_bringup panda_camera_monitor.launch.py robot_ip:=<PANDA_IP>
```

The default home pose is the Panda ready pose from the MoveIt fake-hardware
configuration. Edit `panda/admittance_controller.teach_mode.home_joint_positions_rad`
in `config/controllers.yaml` to change it; the return speed is limited to
`0.30 rad/s` per joint by default.

The controller uses the existing `panda_tcp` force/torque state interfaces,
`panda_hand_tcp` for the sensor/control frame and kinematics tip, and
`panda_link0` as the fixed world frame. The TCP frame matches the installed
Franka stiffness frame K: `F_T_EE` has a -45 degree rotation about Z and a
0.1034 m displacement along Z, and `EE_T_K` is identity. This requires the
hand frames in the URDF (`load_gripper:=true`, the launch default).
All six axes are enabled. The configured `measured_position_command.enabled: true`
selects the measured-position command calculation. The original Humble integration
law remains available with `measured_position_command.enabled: false`.

### Measured-position command calculation

The controller manager and Panda hardware run at 1000 Hz. The admittance
calculation runs at `measured_position_command.update_rate: 100.0` and uses
`measured_position_command.sample_time: 0.01` for its position step.
Between calculations, the target is held and the joint command filter runs
at the hardware rate with the KUKA 40 ms time constant.

At each admittance calculation:

```text
ddx = (F - D * measured_motion - K * displacement_from_entry_pose) / M
twist = clip(ddx * controller_period) * cartesian_gains
dq = joint_gains * damped_SVD_inverse(J) * twist
q_target = q_measured + dq * sample_time
```

Neither `dq` nor the position step is accumulated between calculations.
Measured motion is calculated from consecutive measured TCP poses. The entry
pose and command filter are reset on controller activation. The KUKA source's
rotation convention is preserved literally: angular measured motion is the
angle increment from `current_rotation.inverse() * previous_rotation`, without
division by the controller period. This differs from angular velocity in rad/s.
Orientation quaternions are extracted from Panda's rotation matrix, rather than
using the KUKA source's direct matrix-buffer-to-quaternion construction.

The YAML copies KUKA's mass `[0.01, 0.01, 0.01, 0.01, 0.01, 0.01]`, damping
`[1, 1, 3, 1, 1, 1]`, zero stiffness, Cartesian/joint gains, and SVD damping
lambda `0.2`. `admittance.joint_damping`, `kinematics.alpha`, and the original
`ft_sensor.filter_coefficient` are unused by this command law. Velocity clipping
occurs before gains, matching KUKA; those values are not final joint speed limits.

The base-frame force deadbands are 2 N and torque deadbands are 0.1 Nm. Their
magnitudes are subtracted outside the deadband. Panda still supplies its native
Franka wrench estimate; `measured_position_command.wrench_tau: 0.04` filters that wrench before the
deadband. This replaces KUKA's external-joint-torque filtering and force estimator,
so the force estimation and robot servo remain specific to Panda. Position
commands still feed the chained Cartesian impedance controller, which produces
effort commands. The KUKA Sunrise application also uses Cartesian impedance with
a FRI joint position overlay.

Panda's Cartesian impedance gains are tuned separately in the monitor YAML.
Its quaternion-vector orientation error is approximately half the rotation
angle, so the small-angle rotational stiffness is approximately half the
configured rotational gain. Its damping gains are direct coefficients rather
than Sunrise's Cartesian damping ratio. YAML gain changes are read when the
controller is configured; the compose bind mount makes them available without
rebuilding the image.

This is a C++ implementation change: rebuild the Docker image before restarting
with this YAML. The `measured_position_command.enabled` parameter is read-only after controller creation.
Hardware response must be evaluated separately from the offline command-law tests.

The controller writes position references to the chained Cartesian impedance
controller, which supplies effort commands to the Panda hardware. On Humble, inspect
`/admittance_controller/status` (or `/panda/admittance_controller/status` in
the namespaced launch); `AdmittanceControllerState.joint_state.position`
contains the calculated joint-space offsets.

`gravity_compensation.CoG.force` and `.pos` remain zero because the Franka
external-wrench estimate already excludes the configured end-effector and
payload. Configure those loads on the robot rather than compensating their
weight a second time in admittance. If the robot's `F_T_EE` or `EE_T_K` changes,
update the URDF/configuration so the sensor frame still matches K.

### Panda nullspace posture

The Cartesian impedance controller captures its preferred joint posture on
activation, keeping each target at least `joint_limit_margin` inside the
configured joint limits. Admittance joint references update the TCP pose through
a separate FK cache; they do not overwrite this posture preference. Deactivating
and reactivating captures a new posture from the measured joints.

The posture spring and joint damping are projected using a fixed-size 6x7 SVD.
All numerical null directions are included at rank-deficient configurations
(relative rank tolerance 1e-6, absolute floor 1e-9). This is a kinematic projector;
it does not provide inertia-based dynamic decoupling. The projector can change
when the Jacobian loses rank. The existing total-torque slew limiter bounds
command changes, and the projected posture torque is scaled as a whole to obey
`nullspace_max_torque`, preserving its direction.

`nullspace_damping` is now applied directly in Nm*s/rad, even with zero posture
stiffness. For compliant hand guiding the Panda configuration starts at stiffness
2 Nm/rad, damping 2 Nm*s/rad, and a 2 Nm projected torque bound. These are initial
tuning values, not a claim of critical damping for the robot's actual
configuration-dependent inertia. Hand repositioning works against this gentle
posture spring; after releasing the elbow it is biased toward the captured posture.
Within the 0.15 rad joint-limit margin, a bounded quadratic repulsion
of up to `joint_limit_avoidance_torque` is added before projection. This encourages
clearance where the task permits it; it does not guarantee joint-limit avoidance
when the TCP task conflicts with that objective.

Joint limits and nullspace parameters are read on configuration. Use limits for
the actual robot model; the defaults and monitor YAML are for Panda. C++ changes
require rebuilding the Docker image and recreating the container. YAML tuning
requires reconfiguration or container restart. This posture controller is a
Panda implementation, not a reproduction of Sunrise's internal redundancy law.
