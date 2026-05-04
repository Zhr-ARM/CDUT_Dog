cmake_minimum_required(VERSION 3.8)
project(deep_motor_ros)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# 寻找依赖库
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 包含头文件目录 (假设你把 deep_motor_sdk.h 放在了 include/deep_motor_ros/ 下)
include_directories(include)

# 添加可执行文件
add_executable(motor_node
  src/can_interface_manager.cpp
  src/deep_motor_node.cpp
  src/motor_node.cpp
  src/motor_sdk_bridge.cpp
  src/motor_utils.cpp
)

# 链接依赖
ament_target_dependencies(motor_node rclcpp sensor_msgs std_msgs)

# 安装规则
install(TARGETS
  motor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# 安装 include 目录 (如果有导出头文件的话)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
