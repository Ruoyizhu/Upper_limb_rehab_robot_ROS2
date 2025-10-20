import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    """
    修复版Gazebo启动文件 - 解决spawn_entity服务不可用问题
    """
    
    # 获取包路径
    pkg_path = get_package_share_directory('rehab_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'minimal_robot.urdf')
    
    # 读取URDF文件内容
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # =================================================================
    # 1. 启动Gazebo服务器（关键修复：加载必要的ROS插件）
    # =================================================================
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver', 
            '--verbose', 
            'worlds/empty.world',
            # 关键修复：强制加载ROS插件
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so'
        ],
        output='screen',
        shell=True
    )
    
    # =================================================================
    # 2. 启动机器人状态发布器
    # =================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
        # 关键修复：确保话题名称正确
        remappings=[('robot_description', 'robot_description')]
    )
    
    # =================================================================
    # 3. 在Gazebo中生成机器人实体（关键修复：延迟启动）
    # =================================================================
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'rehab_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '1.0',  # 提高Z坐标确保可见
            '-R', '0.0', '-P', '0.0', '-Y', '0.0'
        ],
        output='screen'
    )
    
    # =================================================================
    # 4. 启动关节状态发布器GUI
    # =================================================================
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # =================================================================
    # 5. 启动Gazebo客户端（GUI）- 可选，如果崩溃可以注释掉
    # =================================================================
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        shell=True
    )
    
    # =================================================================
    # 关键修复：使用事件处理器确保正确的启动顺序
    # =================================================================
    
    # 延迟启动Gazebo客户端，确保服务器先启动
    delayed_gazebo_client = TimerAction(
        period=5.0,  # 等待5秒
        actions=[gazebo_client]
    )
    
    # 延迟启动spawn_entity，确保Gazebo服务器完全启动
    delayed_spawn_entity = TimerAction(
        period=8.0,  # 等待8秒
        actions=[spawn_entity]
    )
    
    # 启动关节状态发布器在机器人状态发布器之后
    delayed_joint_state_publisher = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[joint_state_publisher_gui]
        )
    )
    
    return LaunchDescription([
        # 第一步：启动Gazebo服务器
        gazebo_server,
        
        # 第二步：启动机器人状态发布器
        robot_state_publisher,
        
        # 第三步：延迟启动其他组件
        delayed_gazebo_client,
        delayed_spawn_entity,
        delayed_joint_state_publisher
    ])