cmake_minimum_required(VERSION 3.8)
project(a6_y_teleop_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VrJointReference.msg" "msg/VrControl.msg" "msg/VrState.msg"
  "srv/GravityTuning.srv" "srv/GravityHome.srv"
  "msg/TeachingReference.msg" "srv/TeachingControl.srv")
ament_export_dependencies(rosidl_default_runtime)
ament_package()
