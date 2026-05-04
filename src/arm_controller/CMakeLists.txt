cmake_minimum_required(VERSION 3.8)
project(arm_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(arm_controller_node
  src/arm_controller_node.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
)

target_compile_features(arm_controller_node PUBLIC cxx_std_17)
target_link_libraries(arm_controller_node Eigen3::Eigen)
ament_target_dependencies(arm_controller_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS arm_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
