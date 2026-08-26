cmake_minimum_required(VERSION 3.8)
project(franka_teach_executor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(franka_msgs REQUIRED)
find_package(franka_teach_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

add_executable(franka_teach_executor_node
  src/main.cpp
  src/executor_node.cpp
  src/scoped_allowed_collisions.cpp
)

target_include_directories(franka_teach_executor_node PRIVATE include)

ament_target_dependencies(franka_teach_executor_node
  ament_index_cpp
  franka_msgs
  franka_teach_msgs
  geometry_msgs
  moveit_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

target_link_libraries(franka_teach_executor_node yaml-cpp Eigen3::Eigen ${OpenCV_LIBS} OpenMP::OpenMP_CXX)

install(TARGETS franka_teach_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
