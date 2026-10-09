This package is based on `ros-controls/ros2_controllers` Humble branch at commit
`65db73628ebac1948833f0fbdc60973c74d6cd0e` (admittance_controller 2.54.1).

Local change: adds `admittance.damping`, a direct Cartesian damping coefficient
that can be used with zero stiffness. It is added to the damping derived from
`admittance.damping_ratio` and stiffness.

The optional `measured_position_command` mode uses a force-driven target relative
to measured joints. Its angular displacement and angular velocity are expressed
in the base frame used by the wrench and Jacobian. Angular damping uses rotation
change divided by elapsed time and opposes positive rotation.

The target is filtered using `joint_position_tau`. The final position reference
then follows it with per-joint `max_joint_velocity` (rad/s) and
`max_joint_acceleration` (rad/s²). These seven-element parameters are read-only
while configured; the Panda profile starts with 1 rad/s and 5 rad/s² on each
joint. Reference velocity is the applied position step divided by the hardware
period; acceleration is its velocity change divided by that period. This also
corrects the velocity in `~/joint_commands` and the motion derivatives in
`~/status`, which previously described the unfiltered target calculation.

Limits apply to normal measured-position admittance motion. The separate
return-home trajectory retains its existing limits and command-topic behavior.
These reference bounds are not guarantees on measured physical acceleration or
on a later cubic reconstruction of a recording. Episode replay and policy
execution must still check their fitted trajectories independently.
