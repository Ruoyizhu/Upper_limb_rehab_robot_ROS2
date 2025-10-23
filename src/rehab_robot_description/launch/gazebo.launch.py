import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('rehab_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'minimal_robot.urdf')
    
    # 读取URDF文件
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # 启动Gazebo进程 - 关键修复：添加 -s libgazebo_ros_factory.so
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '/usr/share/gazebo-11/worlds/empty.world'],
        output='screen',
        name='gazebo_server'
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # 修复后的 spawn_entity（移除错误的 -s 参数）
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'rehab_robot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_process,
        robot_state_publisher,
        spawn_entity,
    ])