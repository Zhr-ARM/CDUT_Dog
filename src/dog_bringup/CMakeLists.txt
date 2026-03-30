cmake_minimum_required(VERSION 3.8)
project(dog_bringup)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
set(msg_files "msg/BaseState.msg" "msg/LegState.msg" "msg/SystemMonitor.msg"
              "msg/DogState.msg")

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES
                           geometry_msgs std_msgs)

install(DIRECTORY launch config xacro meshes DESTINATION share/${PROJECT_NAME})

ament_package()
