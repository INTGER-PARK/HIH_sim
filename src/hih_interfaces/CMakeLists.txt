cmake_minimum_required(VERSION 3.8)
project(hih_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/Input.msg"
  "msg/MinitroneState.msg"
  "msg/MobObserverInput.msg"
  "msg/Cmd.msg"
  "msg/Wrench.msg"
  "msg/NormalForce.msg"
  "msg/AttitudeCmd.msg"
  "msg/CenterOfPressure.msg"
  "msg/TorqueDob.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
