cmake_minimum_required(VERSION 3.8)
project(rehab_robot_control)

# 设置默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 设置包含目录
include_directories(include)

# =================================================================
# 1. 创建碗-小球动力学库
# =================================================================
add_library(bowl_ball_dynamics SHARED
  src/bowl_ball_dynamics.cpp
)

# 为库添加依赖
ament_target_dependencies(bowl_ball_dynamics
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

# =================================================================
# 2. 创建主控制节点
# =================================================================
add_executable(robot_control_node
  src/robot_control_node.cpp
)

# 为可执行文件添加依赖
ament_target_dependencies(robot_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

# 链接库到可执行文件
target_link_libraries(robot_control_node bowl_ball_dynamics)

# =================================================================
# 3. 安装配置 - 关键修复部分
# =================================================================

# 安装可执行文件
install(TARGETS robot_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 安装库文件 - 修复：使用标准安装路径
install(TARGETS bowl_ball_dynamics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/
)

# =================================================================
# 4. 导出配置 - 关键修复：移除有问题的export_targets
# =================================================================

# 导出包含目录
ament_export_include_directories(include)

# 导出库
ament_export_libraries(bowl_ball_dynamics)

# 导出依赖
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

# 必须放在最后
ament_package()