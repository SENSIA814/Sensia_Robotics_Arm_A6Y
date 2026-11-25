cmake_minimum_required(VERSION 3.8)

project(y1_6)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes textures urdf)
  install(DIRECTORY ${dir}/
    DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()