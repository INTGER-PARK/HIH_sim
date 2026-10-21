cmake_minimum_required(VERSION 3.8)
project(hih_controller)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================================
# Role-specific controllers.
# Leader/Follower intentionally use separate .cpp files so their code-level
# defaults and future algorithms can be tuned independently.
# ============================================================================

# Wrench controllers
add_executable(leader_wrench_controller src/leader_wrench_controller.cpp)
ament_target_dependencies(
  leader_wrench_controller rclcpp rcl_interfaces hih_interfaces std_msgs)
target_link_libraries(leader_wrench_controller Eigen3::Eigen)

add_executable(follower_wrench_controller src/follower_wrench_controller.cpp)
ament_target_dependencies(
  follower_wrench_controller rclcpp rcl_interfaces hih_interfaces std_msgs)
target_link_libraries(follower_wrench_controller Eigen3::Eigen)

# 6-DoF admittance controllers
add_executable(leader_admittance_controller src/leader_admittance_controller.cpp)
ament_target_dependencies(
  leader_admittance_controller rclcpp rcl_interfaces hih_interfaces std_msgs)
target_link_libraries(leader_admittance_controller Eigen3::Eigen)

add_executable(follower_admittance_controller src/follower_admittance_controller.cpp)
target_compile_features(follower_admittance_controller PRIVATE cxx_std_17)
target_include_directories(follower_admittance_controller PRIVATE include)
ament_target_dependencies(
  follower_admittance_controller rclcpp rcl_interfaces hih_interfaces std_msgs)
target_link_libraries(follower_admittance_controller Eigen3::Eigen)

# Leader normal-axis indirect force controller (standalone ros2 run node)
add_executable(
  leader_normal_force
  src/leader_normal_force.cpp)
ament_target_dependencies(
  leader_normal_force rclcpp hih_interfaces std_msgs)
target_link_libraries(leader_normal_force Eigen3::Eigen)

# Second-order momentum observers
add_executable(
  leader_second_order_momentum_observer
  src/leader_second_order_momentum_observer.cpp)
ament_target_dependencies(
  leader_second_order_momentum_observer rclcpp hih_interfaces)
target_link_libraries(leader_second_order_momentum_observer Eigen3::Eigen)

add_executable(
  follower_second_order_momentum_observer
  src/follower_second_order_momentum_observer.cpp)
ament_target_dependencies(
  follower_second_order_momentum_observer rclcpp hih_interfaces)
target_link_libraries(follower_second_order_momentum_observer Eigen3::Eigen)

# Role-specific CoP attitude-offset controllers. Both publish the existing
# cop_safety attitude interface consumed by the corresponding wrench controller.
add_executable(
  leader_cop_attitude_controller src/leader_cop_attitude_controller.cpp)
target_compile_features(leader_cop_attitude_controller PRIVATE cxx_std_17)
target_include_directories(leader_cop_attitude_controller PRIVATE include)
ament_target_dependencies(
  leader_cop_attitude_controller rclcpp rcl_interfaces hih_interfaces std_msgs)

add_executable(
  follower_cop_attitude_controller src/follower_cop_attitude_controller.cpp)
target_compile_features(follower_cop_attitude_controller PRIVATE cxx_std_17)
target_include_directories(follower_cop_attitude_controller PRIVATE include)
ament_target_dependencies(
  follower_cop_attitude_controller
  rclcpp
  rcl_interfaces
  hih_interfaces
  std_msgs
)

# CoP translational-offset controllers.
add_executable(
  leader_cop_translation_controller src/leader_cop_translation_controller.cpp)
target_compile_features(leader_cop_translation_controller PRIVATE cxx_std_17)
target_include_directories(leader_cop_translation_controller PRIVATE include)
ament_target_dependencies(
  leader_cop_translation_controller rclcpp rcl_interfaces hih_interfaces std_msgs)
target_link_libraries(leader_cop_translation_controller Eigen3::Eigen)

add_executable(
  follower_cop_translation_controller src/follower_cop_translation_controller.cpp)
target_compile_features(follower_cop_translation_controller PRIVATE cxx_std_17)
target_include_directories(follower_cop_translation_controller PRIVATE include)
ament_target_dependencies(
  follower_cop_translation_controller rclcpp rcl_interfaces hih_interfaces std_msgs)
target_link_libraries(follower_cop_translation_controller Eigen3::Eigen)

# ============================================================================
# Shared/non-role-specific nodes kept as-is.
# ============================================================================

add_executable(hih_allocator_controller src/allocator_controller.cpp)
ament_target_dependencies(hih_allocator_controller rclcpp hih_interfaces)
target_link_libraries(hih_allocator_controller Eigen3::Eigen)

# Torque DoB remains one executable that is manually run under a role namespace.
add_executable(hih_torque_dob src/torque_dob.cpp)
target_compile_features(hih_torque_dob PRIVATE cxx_std_17)
target_include_directories(hih_torque_dob PRIVATE include)
ament_target_dependencies(
  hih_torque_dob rclcpp rcl_interfaces hih_interfaces std_msgs)

install(TARGETS
  leader_wrench_controller
  follower_wrench_controller
  leader_admittance_controller
  follower_admittance_controller
  leader_normal_force
  leader_second_order_momentum_observer
  follower_second_order_momentum_observer
  leader_cop_attitude_controller
  follower_cop_attitude_controller
  leader_cop_translation_controller
  follower_cop_translation_controller
  hih_allocator_controller
  hih_torque_dob
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

