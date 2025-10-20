cmake_minimum_required(VERSION 3.8)
project(rehab_robot_description)

# 设置编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)

# =================================================================
# 关键修复：确保所有目录都被正确安装
# =================================================================

# 安装launch目录 - 使用FILES模式确保单个文件被安装
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装urdf目录
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装worlds目录
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 安装config目录（如果存在）
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 必须放在最后
ament_package()