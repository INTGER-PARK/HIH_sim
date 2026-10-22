cmake_minimum_required(VERSION 3.8)
project(hih_cmd)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(hih_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hih_trajectory src/trajectory.cpp)
target_compile_features(hih_trajectory PRIVATE cxx_std_17)
ament_target_dependencies(hih_trajectory rclcpp rcl_interfaces hih_interfaces std_msgs)

add_executable(hih_external_wrench_cmd src/external_wrench_cmd.cpp)
ament_target_dependencies(hih_external_wrench_cmd
  rclcpp
  hih_interfaces
)

install(TARGETS
  hih_trajectory
  hih_external_wrench_cmd
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/hih_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hih_teleop
)
install(PROGRAMS scripts/hih_teleop_leader.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME hih_teleop_leader
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

