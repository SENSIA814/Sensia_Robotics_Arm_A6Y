cmake_minimum_required(VERSION 3.8)
project(a6_y_moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(a6_y_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)

add_executable(end_effector_control_server src/end_effector_control_server.cpp)
target_compile_features(end_effector_control_server PUBLIC cxx_std_17)
ament_target_dependencies(end_effector_control_server
  a6_y_interfaces geometry_msgs moveit_core moveit_ros_planning_interface
  rclcpp rclcpp_action tf2)

install(TARGETS end_effector_control_server DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
