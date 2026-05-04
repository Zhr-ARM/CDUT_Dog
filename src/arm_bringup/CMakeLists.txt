cmake_minimum_required(VERSION 3.8)
project(arm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes models rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/models/arm_bringup
)

ament_package()
