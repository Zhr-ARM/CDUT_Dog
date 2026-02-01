cmake_minimum_required(VERSION 3.8)
project(dog_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf meshes DESTINATION share/${PROJECT_NAME})

ament_package()
