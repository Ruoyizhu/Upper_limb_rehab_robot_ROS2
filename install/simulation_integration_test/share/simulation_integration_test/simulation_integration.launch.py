import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    rehab_robot_description_dir = get_package_share_directory('rehab_robot_description')
    
    # 读取URDF文件
    urdf_file = os.path.join(rehab_robot_description_dir, 'urdf', 'minimal_robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Gazebo启动
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', '--pause', 'worlds/empty.world'],
        output='screen',
        name='gazebo_simulator'
    )
    
    # 生成URDF并加载机器人模型到Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot_entity',
        arguments=['-entity', 'rehab_robot', '-file', urdf_file],
        output='screen'
    )
    
    # 机器人状态发布器
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # =================================================================
    # 关键修复：添加控制器加载节点
    # =================================================================
    
    # 关节状态广播器
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )
    
    # 关节轨迹控制器
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen'
    )
    
    # 其他节点（控制节点、安全节点等）
    robot_control_node = Node(
        package='rehab_robot_control',
        executable='robot_control_node',
        name='robot_control_node',
        output='screen',
        remappings=[
            ('/control_cmd', '/joint_trajectory_controller/joint_trajectory')  # 关键重映射
        ]
    )
    
    safety_monitor_node = Node(
        package='safety_monitor',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen'
    )
    
    test_teleop_node = Node(
        package='simulation_integration_test',
        executable='test_teleop_node.py',
        name='test_teleop_node',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_process,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gazebo_process,
                on_exit=[spawn_entity_node]
            )
        ),
        robot_state_publisher_node,
        # 等待机器人加载后启动控制器
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[joint_state_broadcaster]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[joint_trajectory_controller]
            )
        ),
        # 其他节点
        robot_control_node,
        safety_monitor_node,
        test_teleop_node
    ])