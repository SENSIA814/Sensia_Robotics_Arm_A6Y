cmake_minimum_required(VERSION 3.8)
project(a6_y_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(a6_y_damiao_system SHARED
  src/a6_y_damiao_system.cpp
  src/damiao_can.cpp
)
target_compile_features(a6_y_damiao_system PUBLIC cxx_std_17)
target_include_directories(a6_y_damiao_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(a6_y_damiao_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_executable(damiao_protocol_check
  src/damiao_protocol_check.cpp
  src/damiao_can.cpp
)
target_compile_features(damiao_protocol_check PUBLIC cxx_std_17)
target_include_directories(damiao_protocol_check PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(hardware_interface a6_y_ros2_control_plugin.xml)

install(
  TARGETS a6_y_damiao_system
  EXPORT export_a6_y_damiao_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS damiao_protocol_check
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch tools "Motor Drive" DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  tools/apply_zero_offsets.py
  tools/capture_zero_offsets.py
  tools/damiao_bus_scan.py
  tools/motor_status.py
  tools/move_to_group_state.py
  tools/read_motor_params.py
  tools/save_motor_params_flash.py
  tools/single_motor_check.py
  tools/validate_motor_config.py
  tools/write_motor_params_temp.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES a6_y_ros2_control_plugin.xml README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_a6_y_damiao_system HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
