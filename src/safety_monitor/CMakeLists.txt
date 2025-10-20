cmake_minimum_required(VERSION 3.8)
project(safety_monitor)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 设置包含目录
include_directories(include)

# 创建可执行文件 - 添加 safety_checker.cpp
add_executable(safety_monitor_node
  src/safety_monitor_node.cpp
  src/safety_checker.cpp  # 添加这一行
)

# 添加依赖
ament_target_dependencies(safety_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
)

# 安装目标文件
install(TARGETS
  safety_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件和配置
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()