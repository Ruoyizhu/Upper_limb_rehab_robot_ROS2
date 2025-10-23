# =================================================================
# CMake 基础配置部分
# =================================================================

# 指定CMake的最低版本要求，确保使用的特性在该版本中可用[3](@ref)
# VERSION 3.8 表示需要CMake 3.8或更高版本
cmake_minimum_required(VERSION 3.8)

# 定义项目名称和相关信息[1,3](@ref)
# rehab_robot_control: 项目名称，会在编译过程中使用
project(rehab_robot_control)

# 设置默认构建类型（如果用户没有明确指定）[4](@ref)
# Release模式会进行优化，适合最终部署；Debug模式包含调试信息，适合开发阶段
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)  # 设置为Release模式以获得最佳性能
endif()

# 设置编译器警告选项，提高代码质量[5](@ref)
# 检查编译器是否为GCC或Clang（这两个编译器支持这些警告选项）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)  # 启用所有警告、额外警告和严格标准检查
endif()

# 设置C++标准为C++17[4,9](@ref)
# C++17提供了现代C++特性，如结构化绑定、if初始化语句等
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 强制要求使用C++17标准，如果编译器不支持会报错

# =================================================================
# 依赖包查找配置
# =================================================================

# 查找ROS 2的核心构建工具包（必须）[9](@ref)
find_package(ament_cmake REQUIRED)

# 查找ROS 2 C++客户端库（核心通信功能）[9](@ref)
find_package(rclcpp REQUIRED)

# 查找传感器消息包（用于关节状态等传感器数据）
find_package(sensor_msgs REQUIRED)

# 查找几何消息包（用于向量、点等几何数据）
find_package(geometry_msgs REQUIRED)

# 查找标准消息包（基础消息类型）
find_package(std_msgs REQUIRED)

# 查找TF2库（坐标系变换功能）
find_package(tf2 REQUIRED)

# 查找TF2 ROS集成包（ROS 2专用的TF2功能）
find_package(tf2_ros REQUIRED)

# 设置头文件包含目录[4](@ref)
# 将项目的include目录添加到编译器的头文件搜索路径中
include_directories(include)

# =================================================================
# 库和可执行文件构建配置
# =================================================================

# 创建碗-小球动力学共享库[4,7](@ref)
# SHARED表示创建动态链接库（.so文件），可以在运行时被多个程序共享
add_library(bowl_ball_dynamics SHARED
  src/bowl_ball_dynamics.cpp  # 库的源文件，包含核心算法实现
)

# 为动力学库添加依赖关系[9](@ref)
# 这些依赖会在编译和链接时被自动处理
ament_target_dependencies(bowl_ball_dynamics
  rclcpp        # ROS 2 C++客户端库
  sensor_msgs   # 传感器消息支持
  geometry_msgs # 几何消息支持
  std_msgs      # 标准消息支持
  tf2           # 坐标系变换库
  tf2_ros       # TF2 ROS集成
)

# 创建主控制节点可执行文件[3](@ref)
add_executable(robot_control_node
  src/robot_control_node.cpp  # 主控制节点的源文件
)

# 为控制节点添加依赖关系
ament_target_dependencies(robot_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

# 将动力学库链接到控制节点[4](@ref)
# 这样控制节点就可以使用库中定义的函数和类
target_link_libraries(robot_control_node bowl_ball_dynamics)

# =================================================================
# 安装配置 - 关键修复部分
# =================================================================

# 安装可执行文件到指定目录[7,8](@ref)
# RUNTIME DESTINATION: 可执行文件的安装目录
install(TARGETS robot_control_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}  # 安装到lib/rehab_robot_control目录
)

# 安装库文件 - 使用标准安装路径（修复之前的问题）[7](@ref)
install(TARGETS bowl_ball_dynamics
  ARCHIVE DESTINATION lib    # 静态库文件（.a）安装目录
  LIBRARY DESTINATION lib    # 动态库文件（.so）安装目录
  RUNTIME DESTINATION bin    # Windows平台DLL文件安装目录
)

# 安装头文件供其他包使用[7](@ref)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}  # 安装到include/rehab_robot_control目录
)

# 安装启动文件[7](@ref)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/  # 安装到share/rehab_robot_control/launch目录
)

# 安装配置文件[7](@ref)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/  # 安装到share/rehab_robot_control/config目录
)

# =================================================================
# 导出配置 - 确保其他包能正确找到本包
# =================================================================

# 导出包含目录，这样其他包在find_package时能自动找到头文件[9](@ref)
ament_export_include_directories(include)

# 导出库文件，这样其他包可以链接到这个库[9](@ref)
ament_export_libraries(bowl_ball_dynamics)

# 导出依赖关系，确保本包的依赖也能传递给使用本包的其他包[9](@ref)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

# =================================================================
# 必须放在CMakeLists.txt的最后[9](@ref)
# 这个命令会生成必要的安装脚本和包配置文件
# =================================================================
ament_package()