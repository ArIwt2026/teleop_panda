# Franka bringup controllers

## Panda admittance controller (ROS 2 Humble)

The Humble admittance controller is registered in `config/controllers.yaml` and
can be started with:

```bash
ros2 launch franka_bringup franka.launch.py \
  robot_ip:=<PANDA_IP> load_admittance_controller:=true
```

For the namespaced camera monitor launch, pass the same flag:

```bash
ros2 launch franka_bringup panda_camera_monitor.launch.py \
  robot_ip:=<PANDA_IP> load_admittance_controller:=true
```

The controller uses the existing `panda_tcp` force/torque state interfaces,
`panda_hand_tcp` for the sensor/control frame and kinematics tip, and
`panda_link0` as the fixed world frame. The TCP frame matches the installed
Franka stiffness frame K: `F_T_EE` has a -45 degree rotation about Z and a
0.1034 m displacement along Z, and `EE_T_K` is identity. This requires the
hand frames in the URDF (`load_gripper:=true`, the launch default).
Translation axes are enabled; rotational axes are disabled. Admittance
stiffness and damping ratio are configured to zero, and joint damping is 5.0.
The virtual translational mass is 5 kg per axis.

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
