Direct Panda joint-limit resistance
==================================

The direct launch loads joint_limits.yaml. The seven position limits come from
the Panda/FER specification and match the local panda_arm.xacro. All arrays use
panda_joint1 through panda_joint7 order. Positions are radians; stiffness is
Nm/rad, damping Nm/(rad/s), and torque caps Nm. Missing or invalid configuration
prevents node startup.

Resistance blends smoothly into the existing Coriolis torque near each limit.
Directional damping opposes approach and releases on retreat. A 0.1 second
velocity preview activates resistance earlier for faster approaches. This is
not a model-based stopping-distance guarantee. Libfranka torque rate limiting
and 100 Hz filtering remain enabled.

Initial commissioning settings: 10 degree activation distance, 3 degree inner
reserve. The reserve sets full wall blending; crossing it keeps control active
so the operator can retreat. Startup inside the reserve is allowed while inside
the physical limits and free of robot errors. Restoring torque assists retreat;
directional damping releases on retreat. At the configured physical limits or
invalid state, the callback aborts. The robot's own protections remain active.
Resistance exists only in torque mode. Libfranka rate limiting and filtering
smooth startup torque, including when starting within the wall.

Current robot errors and non-idle initial robot modes block torque startup.
After a fault, the worker waits for a new toggle; it does not perform automatic
error recovery. Stale joint-state publication stops on session failure.

Before hardware use, verify payload and exact robot limits. Commission at low
speed with temporary limits well inside the physical range. Measure stopping
overshoot, retreat behavior, multi-joint interaction, and saturation. Adjust
per-joint gains and margins based on those results. A human can overcome the
capped resistance; the 3 degree reserve is not guaranteed containment.

Offline guard check:
  g++ -std=c++17 test_joint_limit_guard.cpp -o /tmp/joint_limit_guard_test
  /tmp/joint_limit_guard_test

The standalone executable now also requires the YAML parameter file, e.g.
  --ros-args -r __ns:=/panda --params-file <package-share>/config/joint_limits.yaml
