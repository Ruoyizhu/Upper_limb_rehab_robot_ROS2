#!/usr/bin/env python3
"""
测试遥控节点：模拟患者运动意图
通过键盘输入发布患者意图消息，用于集成测试康复机器人系统
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import threading
import sys
import select
import tty
import termios

class TestTeleopNode(Node):
    def __init__(self):
        super().__init__('test_teleop_node')
        
        # 创建发布者，发布患者意图
        self.publisher_ = self.create_publisher(Vector3, '/patient_intent', 10)
        
        # 设置控制参数
        self.control_gain = 0.1  # 控制增益系数
        self.max_intent = 1.0   # 最大意图值
        
        # 患者意图数据
        self.intent_x = 0.0
        self.intent_y = 0.0
        self.intent_z = 0.0
        
        # 创建定时器发布意图
        self.timer_period = 0.1  # 10Hz发布频率
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 键盘输入处理
        self.settings = termios.tcgetattr(sys.stdin)
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.get_logger().info('测试遥控节点已启动')
        self.get_logger().info('使用WASD键控制方向，空格键重置，Q键退出')
        self.get_logger().info('W: 向前, S: 向后, A: 向左, D: 向右')
        
    def timer_callback(self):
        """定时发布患者意图消息"""
        msg = Vector3()
        msg.x = self.intent_x
        msg.y = self.intent_y
        msg.z = self.intent_z
        
        self.publisher_.publish(msg)
        self.get_logger().debug('发布患者意图: x=%.3f, y=%.3f, z=%.3f' % (msg.x, msg.y, msg.z))
    
    def keyboard_listener(self):
        """监听键盘输入并更新患者意图"""
        while rclpy.ok():
            key = self.get_key()
            self.process_key_input(key)
    
    def process_key_input(self, key):
        """处理键盘输入"""
        if key == 'q' or key == 'Q':
            # 退出程序
            self.get_logger().info('退出测试节点')
            rclpy.shutdown()
            return
        elif key == 'w' or key == 'W':
            # 向前运动意图
            self.intent_x = self.control_gain
            self.intent_y = 0.0
        elif key == 's' or key == 'S':
            # 向后运动意图
            self.intent_x = -self.control_gain
            self.intent_y = 0.0
        elif key == 'a' or key == 'A':
            # 向左运动意图
            self.intent_x = 0.0
            self.intent_y = self.control_gain
        elif key == 'd' or key == 'D':
            # 向右运动意图
            self.intent_x = 0.0
            self.intent_y = -self.control_gain
        elif key == ' ':
            # 空格键重置意图
            self.intent_x = 0.0
            self.intent_y = 0.0
            self.intent_z = 0.0
            self.get_logger().info('重置患者意图为零')
        elif key == '+':
            # 增加控制增益
            self.control_gain = min(self.control_gain + 0.05, self.max_intent)
            self.get_logger().info('控制增益增加至: %.2f' % self.control_gain)
        elif key == '-':
            # 减小控制增益
            self.control_gain = max(self.control_gain - 0.05, 0.05)
            self.get_logger().info('控制增益减小至: %.2f' % self.control_gain)
        else:
            # 忽略其他按键
            return
        
        self.get_logger().info('键盘输入: %s, 当前意图: x=%.2f, y=%.2f, 增益: %.2f' % 
                              (key, self.intent_x, self.intent_y, self.control_gain))
    
    def get_key(self):
        """获取单个键盘输入（非阻塞）"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def destroy_node(self):
        """节点销毁时恢复终端设置"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TestTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()