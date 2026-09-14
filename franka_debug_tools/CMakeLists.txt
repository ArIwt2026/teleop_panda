cmake_minimum_required(VERSION 3.10)
project(franka_debug_tools)

set(CMAKE_CXX_STANDARD 17)

# ROS2 / Ament
find_package(ament_cmake REQUIRED)

# Find libfranka package
find_package(Franka QUIET)
if(NOT Franka_FOUND)
  set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/build/install")
  find_package(Franka REQUIRED)
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find realsense
find_package(realsense2 REQUIRED)

# Find Eigen
find_package(Eigen3 REQUIRED)

# Find yaml-cpp
find_package(yaml-cpp REQUIRED)

add_executable(echo_robot_state echo_robot_state.cpp)
target_link_libraries(echo_robot_state PRIVATE Franka::Franka)

add_library(check_fci SHARED check_fci.cpp)
target_link_libraries(check_fci PRIVATE Franka::Franka)

add_executable(test_gripper test_gripper.cpp)
target_link_libraries(test_gripper PRIVATE Franka::Franka)

# Impedance control (Cartesian impedance controller)
set(LIBFRANKA_EXAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../libfranka/examples")
add_executable(impedance_control
  impedance_control.cpp
  ${LIBFRANKA_EXAMPLES_DIR}/examples_common.cpp
)
target_include_directories(impedance_control PRIVATE ${LIBFRANKA_EXAMPLES_DIR})
target_link_libraries(impedance_control PRIVATE Franka::Franka Eigen3::Eigen)

add_executable(zero_torque_control
  zero_torque_control.cpp
)
target_link_libraries(zero_torque_control PRIVATE Franka::Franka)

# Find ROS packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_teach_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(direct_cartesian_impedance
  direct_cartesian_impedance_node.cpp
  ${LIBFRANKA_EXAMPLES_DIR}/examples_common.cpp
)
target_include_directories(direct_cartesian_impedance PRIVATE ${LIBFRANKA_EXAMPLES_DIR})
target_link_libraries(direct_cartesian_impedance Franka::Franka Eigen3::Eigen)
ament_target_dependencies(direct_cartesian_impedance rclcpp sensor_msgs std_srvs)

# Install executables and libraries
set(DEBUG_TARGETS echo_robot_state test_gripper impedance_control direct_cartesian_impedance zero_torque_control check_fci)
install(TARGETS ${DEBUG_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts/fault manager
install(PROGRAMS
  franka_fault_manager.py
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
