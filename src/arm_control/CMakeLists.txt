cmake_minimum_required(VERSION 3.8)
project(arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ---------------------------------------------------------------------------
# Service interfaces
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlanMove.srv"
)

# ---------------------------------------------------------------------------
# ros2_control hardware plugin
# ---------------------------------------------------------------------------
add_library(arm_system_hardware SHARED
  src/arm_system_hardware.cpp
)

set(ARM_CONTROL_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

target_include_directories(arm_system_hardware PUBLIC
  # Used while building this workspace.
  $<BUILD_INTERFACE:${ARM_CONTROL_INCLUDE_DIR}>
  # Used after this package is installed.
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  arm_system_hardware
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface
  arm_hardware_plugins.xml
)

# ---------------------------------------------------------------------------
# Executables
# ---------------------------------------------------------------------------
add_executable(target_to_moveit src/arm_controller.cpp)

ament_target_dependencies(
  target_to_moveit
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(moveit_plan_service src/moveit_plan_service.cpp)

ament_target_dependencies(
  moveit_plan_service
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  moveit_ros_planning_interface
  moveit_msgs
)

add_executable(dm_j4340_scan_node src/dm_j4340_scan_node.cpp)

target_include_directories(dm_j4340_scan_node PUBLIC
  $<BUILD_INTERFACE:${ARM_CONTROL_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  dm_j4340_scan_node
  rclcpp
  std_msgs
)

add_executable(box_motion_node src/box_motion_node.cpp)

ament_target_dependencies(
  box_motion_node
  rclcpp
  std_msgs
  trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(target_to_moveit "${cpp_typesupport_target}")
target_link_libraries(moveit_plan_service "${cpp_typesupport_target}")
target_link_libraries(box_motion_node "${cpp_typesupport_target}")

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS arm_system_hardware
  EXPORT export_arm_system_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  target_to_moveit
  moveit_plan_service
  dm_j4340_scan_node
  box_motion_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_arm_system_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
)

ament_package()
