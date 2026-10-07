This package is based on `ros-controls/ros2_controllers` Humble branch at commit
`65db73628ebac1948833f0fbdc60973c74d6cd0e` (admittance_controller 2.54.1).

Local change: adds `admittance.damping`, a direct Cartesian damping coefficient
that can be used with zero stiffness. It is added to the damping derived from
`admittance.damping_ratio` and stiffness.
