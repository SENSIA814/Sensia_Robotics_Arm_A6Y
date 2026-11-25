cmake_minimum_required(VERSION 3.8)
project(a6_y_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio 4.0 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(a6_y_teleop_msgs REQUIRED)

add_library(a6_y_gravity_comp_controller SHARED
  src/A6_y_gravity_comp_controller.cpp
  src/gravity_home.cpp
  src/gravity_follow.cpp
  src/A6_y_cartesian_impedance_controller.cpp
  src/A6_y_joint_impedance_controller.cpp
)
target_compile_features(a6_y_gravity_comp_controller PUBLIC cxx_std_17)
target_include_directories(a6_y_gravity_comp_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(a6_y_gravity_comp_controller
  ament_index_cpp
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  a6_y_teleop_msgs
)
target_link_libraries(a6_y_gravity_comp_controller
  pinocchio::pinocchio
)

pluginlib_export_plugin_description_file(controller_interface A6_y_controllers_plugin.xml)

install(
  TARGETS a6_y_gravity_comp_controller
  EXPORT export_a6_y_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES A6_y_controllers_plugin.xml README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_a6_y_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  controller_interface
  hardware_interface
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  a6_y_teleop_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_joint_reference_gate test/test_joint_reference_gate.cpp)
  target_include_directories(test_joint_reference_gate PRIVATE include)
  target_compile_features(test_joint_reference_gate PRIVATE cxx_std_17)
  ament_target_dependencies(test_joint_reference_gate a6_y_teleop_msgs)
  ament_add_gtest(test_smooth_friction_compensation
    test/test_smooth_friction_compensation.cpp
  )
  target_include_directories(test_smooth_friction_compensation PRIVATE include)
  target_compile_features(test_smooth_friction_compensation PRIVATE cxx_std_17)
  ament_add_gtest(test_haptic_effort_composer
    test/test_haptic_effort_composer.cpp
  )
  target_include_directories(test_haptic_effort_composer PRIVATE include)

  ament_add_gtest(test_safe_zero_effort_command
    test/test_safe_zero_effort_command.cpp
  )
  target_include_directories(test_safe_zero_effort_command PRIVATE include)

  ament_add_gtest(test_a5_gravity_controller test/test_a5_gravity_controller.cpp)
  target_link_libraries(test_a5_gravity_controller a6_y_gravity_comp_controller)
  target_compile_definitions(test_a5_gravity_controller PRIVATE
    A5_TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../A5/urdf/A5.urdf")

  ament_add_gtest(test_a5_cartesian_impedance_controller test/test_a5_cartesian_impedance_controller.cpp)
  target_link_libraries(test_a5_cartesian_impedance_controller a6_y_gravity_comp_controller)
  target_compile_definitions(test_a5_cartesian_impedance_controller PRIVATE
    A5_TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../A5/urdf/A5.urdf")

  ament_add_gtest(test_bounded_joint_impedance test/test_bounded_joint_impedance.cpp)
  target_link_libraries(test_bounded_joint_impedance a6_y_gravity_comp_controller)

  ament_add_gtest(test_gravity_home_brake test/test_gravity_home_brake.cpp)
  target_include_directories(test_gravity_home_brake PRIVATE include)
  target_compile_features(test_gravity_home_brake PRIVATE cxx_std_17)

  ament_add_gtest(test_a5_joint_impedance_controller test/test_a5_joint_impedance_controller.cpp)
  target_link_libraries(test_a5_joint_impedance_controller a6_y_gravity_comp_controller)
  target_compile_definitions(test_a5_joint_impedance_controller PRIVATE
    A5_TEST_CONFIG_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/a5_joint_impedance_controllers.yaml"
    A5_TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../A5/urdf/A5.urdf")

endif()

ament_package()
