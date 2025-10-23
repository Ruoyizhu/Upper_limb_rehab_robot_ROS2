#!/usr/bin/env python3
"""
安全监控系统ROS2启动文件
该文件用于启动安全监控节点，并加载相应的配置参数
主要功能：初始化安全监控节点、加载参数文件、配置节点行为
"""

# 导入操作系统接口模块，用于文件路径操作
import os

# 从ament_index_python包导入获取包共享目录的函数
# 该函数用于定位ROS2功能包的share目录路径
from ament_index_python.packages import get_package_share_directory

# 导入LaunchDescription类，用于描述启动文件内容
from launch import LaunchDescription

# 导入Node类，用于声明和配置ROS2节点
from launch_ros.actions import Node


def generate_launch_description():
    """
    生成启动描述的主函数
    ROS2 launch系统会自动调用此函数来获取启动配置
    返回: LaunchDescription对象，包含所有要启动的节点和配置
    """
    
    # 获取功能包的共享目录路径
    # 'safety_monitor'是功能包名称，此路径通常包含config、launch等目录
    pkg_path = get_package_share_directory('safety_monitor')
    
    # 构建参数文件的完整路径
    # 参数文件通常包含安全监控器的各种阈值和配置参数
    params_file = os.path.join(pkg_path, 'config', 'safety_params.yaml')
    
    # 创建安全监控节点配置
    # Node类用于定义要启动的ROS2节点及其参数
    safety_monitor_node = Node(
        # 指定节点所在的功能包名称
        package='safety_monitor',
        
        # 指定要执行的可执行文件名称
        # 这对应CMakeLists.txt或setup.py中定义的可执行目标
        executable='safety_monitor_node',
        
        # 设置节点名称，在ROS2系统中用于标识该节点
        # 如果不指定，默认使用可执行文件名称
        name='safety_monitor_node',
        
        # 设置节点输出目标
        # 'screen'表示输出到当前终端屏幕，便于调试和监控
        # 其他选项：'log'（输出到日志文件）或None（默认输出）
        output='screen',
        
        # 指定要加载的参数文件列表
        # 可以传入多个参数文件，后者会覆盖前者的同名参数
        parameters=[params_file]
        
        # 其他可用参数示例（根据需要添加）:
        # namespace='safety',  # 设置节点命名空间
        # arguments=['--debug', '--verbose'],  # 传递给可执行文件的命令行参数
        # remappings=[  # 主题重映射，用于修改主题名称
        #     ('/input_topic', '/safety/input'),
        #     ('/output_topic', '/safety/output')
        # ],
        # respawn=True,  # 节点异常退出时自动重启
        # respawn_delay=3.0,  # 重启延迟时间（秒）
    )
    
    # 创建并返回启动描述对象
    # 将所有要启动的节点添加到LaunchDescription中
    return LaunchDescription([
        safety_monitor_node,
        # 可以在此添加更多节点，例如：
        # logger_node,
        # monitor_node,
    ])


# 条件执行部分：当直接运行此脚本时进行测试
if __name__ == '__main__':
    # 输出提示信息，说明如何正确使用launch文件
    print("该文件应使用 'ros2 launch' 命令运行:")
    print("ros2 launch safety_monitor safety_monitor.launch.py")
    
    # 可选：在此添加简单的功能测试代码
    # 例如验证参数文件是否存在
    if os.path.exists(params_file):
        print(f"参数文件路径正确: {params_file}")
    else:
        print(f"警告: 参数文件不存在: {params_file}")