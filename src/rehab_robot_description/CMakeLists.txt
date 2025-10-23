cmake_minimum_required(VERSION 3.8)
project(rehab_robot_description)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)

# 关键修复：显式安装URDF文件
install(FILES
  urdf/minimal_robot.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装启动文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()