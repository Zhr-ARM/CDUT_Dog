cmake_minimum_required(VERSION 3.8)
project(dog_controllers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 测试控制插件
add_library(dog_test_controller SHARED src/dog_test_controller.cpp)
target_include_directories(dog_test_controller PRIVATE include)
ament_target_dependencies(dog_test_controller rclcpp controller_interface
                          hardware_interface)
pluginlib_export_plugin_description_file(controller_interface
                                         dog_controllers.xml)

# 安装规则
install(TARGETS dog_test_controller DESTINATION lib)
install(FILES dog_controllers.xml DESTINATION share/${PROJECT_NAME})

ament_package()
