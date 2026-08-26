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
set(LIBFRANKA_EXAMPLES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../libfranka/examples")
add_executable(impedance_control
  impedance_control.cpp
  ${LIBFRANKA_EXAMPLES_DIR}/examples_common.cpp
)
target_include_directories(impedance_control PRIVATE ${LIBFRANKA_EXAMPLES_DIR})
target_link_libraries(impedance_control PRIVATE Franka::Franka Eigen3::Eigen)

# Find ROS packages
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(franka_teach_msgs REQUIRED)

# Snap7 Configuration
set(SNAP7_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../external/snap7")
set(SNAP7_INCLUDE_DIR "${SNAP7_ROOT}/release/wrappers/c-cpp")
set(SNAP7_SOURCE "${SNAP7_ROOT}/release/wrappers/c-cpp/snap7.cpp")
set(SNAP7_LIBRARY "${SNAP7_ROOT}/build/bin/linux/libsnap7.so")

add_executable(plc_bridge plc_bridge.cpp ${SNAP7_SOURCE})
target_include_directories(plc_bridge PRIVATE ${SNAP7_INCLUDE_DIR})
target_link_libraries(plc_bridge PRIVATE ${SNAP7_LIBRARY} pthread rt)

# Set RPATH so the executable can find libsnap7.so in the same directory
set_target_properties(plc_bridge PROPERTIES 
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "$ORIGIN"
)

# Install executables and libraries
install(TARGETS echo_robot_state test_gripper impedance_control plc_bridge check_fci
  DESTINATION lib/${PROJECT_NAME}
)

# Install Snap7 library to the same directory as the executable
install(FILES ${SNAP7_LIBRARY}
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts/fault manager
install(PROGRAMS
  franka_fault_manager.py
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
