import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径和参数文件
    pkg_path = get_package_share_directory('safety_monitor')
    params_file = os.path.join(pkg_path, 'config', 'safety_params.yaml')
    
    # 创建安全监控节点
    safety_monitor_node = Node(
        package='safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        safety_monitor_node,
    ])