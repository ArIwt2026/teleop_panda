cmake_minimum_required(VERSION 3.5)
project(franka_teach_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToCameraPose.action"
  "action/ExecuteRelativePoint.action"
  "action/PickPlace.action"
  "action/PlayPoint.action"
  "action/PlaySequence.action"
  "msg/ExecutorStatus.msg"
  "srv/GetExecutorStatus.srv"
  "srv/SelectTarget.srv"
  "srv/PlanTarget.srv"
  "srv/ExecuteTarget.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
