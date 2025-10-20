import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径和参数文件
    pkg_path = get_package_share_directory('rehab_robot_control')
    params_file = os.path.join(pkg_path, 'config', 'control_params.yaml')
    
    # 创建控制节点
    control_node = Node(
        package='rehab_robot_control',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        parameters=[params_file]
    )
    
    return LaunchDescription([
        control_node,
    ])