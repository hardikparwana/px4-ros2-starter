cmake_minimum_required(VERSION 3.5)
project(quad_navigation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread date_time)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(vicon_receiver REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(vicon_px4_bridge nodes/vicon_px4_bridge.cpp)
ament_target_dependencies(vicon_px4_bridge rclcpp std_msgs px4_msgs vicon_receiver)
install(TARGETS
  vicon_px4_bridge
  DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_control nodes/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs geometry_msgs)
install(TARGETS
  offboard_control
  DESTINATION lib/${PROJECT_NAME})

add_executable(offboard_control_external nodes/offboard_external_control.cpp)
ament_target_dependencies(offboard_control_external rclcpp px4_msgs geometry_msgs)
install(TARGETS
  offboard_control_external
  DESTINATION lib/${PROJECT_NAME})

add_executable(quad_controller nodes/quad_cbf_control.cpp)
ament_target_dependencies(quad_controller rclcpp px4_msgs Eigen3)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)
install(TARGETS
  quad_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()