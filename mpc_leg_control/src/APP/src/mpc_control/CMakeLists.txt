# 最低 CMake 版本要求
cmake_minimum_required(VERSION 3.8)

# 工程名（ROS2 包名保持一致）
project(mpc_control)

# 对 GCC/Clang 开启常用警告，帮助发现潜在问题
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++17（代码里用了 std::clamp 等特性）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找构建与运行依赖
# ament_cmake: ROS2 CMake 构建系统
# rclcpp: C++ ROS2 客户端库
# std_msgs: 标准消息类型（发布 motor_cmd / 订阅 gait_mode）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 生成可执行程序
add_executable(mpc_gait_controller src/mpc_gait_controller.cpp)

# 声明目标依赖（自动处理 include、链接和导出）
ament_target_dependencies(mpc_gait_controller rclcpp std_msgs sensor_msgs)

# 安装可执行文件到 ROS2 约定目录
install(TARGETS
  mpc_gait_controller
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置和启动文件，供 `ros2 launch` 与参数加载使用
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装 README，方便在安装空间查阅说明
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

# 可选：测试与 lint
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 当前项目暂时跳过版权与 cpplint 的严格检查
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 生成 ament 包导出信息（必须）
ament_package()
