# -*- coding: utf-8 -*-
"""
康复机器人控制节点启动文件
功能：用于启动康复机器人系统的核心控制节点
作者：Your Name
日期：2024-01-01
版本：1.0
"""

import os  # 提供与操作系统交互的功能，用于文件路径操作和系统环境变量访问[3](@ref)
from ament_index_python.packages import get_package_share_directory  # 从ament_index_python包导入函数，用于获取ROS 2功能包的共享目录路径[6](@ref)
from launch import LaunchDescription  # 导入LaunchDescription类，用于定义launch文件的内容和结构[7](@ref)
from launch_ros.actions import Node  # 导入Node动作类，用于配置和启动ROS 2节点[7](@ref)

def generate_launch_description():
    """
    生成launch描述的主要函数[7](@ref)
    
    这是ROS 2 launch文件的入口函数，用于创建并返回一个LaunchDescription对象。
    该对象包含要启动的节点、参数配置和其他launch动作[8](@ref)。
    
    Returns:
        LaunchDescription: 包含节点启动配置的launch描述对象，用于ros2 launch命令执行
    """
    # 获取康复机器人控制功能包的共享目录路径
    # 使用get_package_share_directory函数，通过包名'rehab_robot_control'定位包路径[6](@ref)
    pkg_path = get_package_share_directory('rehab_robot_control')
    
    # 构建参数文件的完整路径
    # 参数文件通常包含节点的配置参数，如控制算法参数、安全阈值等[8](@ref)
    # os.path.join用于跨平台安全地拼接路径片段[3](@ref)
    params_file = os.path.join(pkg_path, 'config', 'control_params.yaml')
    
    # 创建控制节点动作实例
    # 使用Node类来定义节点启动参数，这是启动ROS 2节点的核心配置[7](@ref)
    control_node = Node(
        package='rehab_robot_control',  # 指定节点所在的功能包名[6](@ref)
        executable='robot_control_node',  # 指定要执行的可执行文件名称（编译后生成的二进制文件）[7](@ref)
        name='robot_control_node',  # 设置节点名称，在ROS图中唯一标识该节点[6](@ref)
        output='screen',  # 控制节点输出方式，'screen'表示将输出显示在终端上，便于调试和监控[7](@ref)
        parameters=[params_file]  # 加载参数文件，将YAML文件中定义的参数传递到节点[8](@ref)
    )
    
    # 返回LaunchDescription对象，包含要启动的节点列表
    # 多个节点可以以列表形式添加，此处只启动一个控制节点[7](@ref)
    return LaunchDescription([
        control_node,  # 将控制节点添加到launch描述中
    ])