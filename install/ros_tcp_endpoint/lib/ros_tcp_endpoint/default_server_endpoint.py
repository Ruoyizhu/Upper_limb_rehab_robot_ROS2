#!/usr/bin/env python3
"""
ROS2 TCP Endpoint默认服务器
用于与Unity的ROS-TCP-Connector通信
"""

import rclpy
from rclpy.node import Node
import socket
import threading
import json

class ROSTCPEndpoint(Node):
    def __init__(self):
        super().__init__('ros_tcp_endpoint')
        self.get_logger().info('ROS2 TCP Endpoint启动')
        
        # TCP服务器配置
        self.host = '0.0.0.0'  # 监听所有接口
        self.port = 10000      # 默认端口
        
        self.setup_tcp_server()
    
    def setup_tcp_server(self):
        """设置TCP服务器"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            self.get_logger().info(f'TCP服务器监听 {self.host}:{self.port}')
            
            # 启动接受连接线程
            self.accept_thread = threading.Thread(target=self.accept_connections)
            self.accept_thread.daemon = True
            self.accept_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'TCP服务器设置失败: {e}')
    
    def accept_connections(self):
        """接受客户端连接"""
        while rclpy.ok():
            try:
                client_socket, address = self.socket.accept()
                self.get_logger().info(f'接受来自 {address} 的连接')
                
                # 为每个客户端启动处理线程
                client_thread = threading.Thread(
                    target=self.handle_client, 
                    args=(client_socket, address)
                )
                client_thread.daemon = True
                client_thread.start()
                
            except Exception as e:
                self.get_logger().error(f'接受连接错误: {e}')
    
    def handle_client(self, client_socket, address):
        """处理客户端通信"""
        try:
            while rclpy.ok():
                # 接收数据（简单实现）
                data = client_socket.recv(1024)
                if not data:
                    break
                
                # 处理接收到的数据（这里需要根据您的协议实现）
                self.process_received_data(data, client_socket)
                
        except Exception as e:
            self.get_logger().error(f'处理客户端 {address} 错误: {e}')
        finally:
            client_socket.close()
            self.get_logger().info(f'客户端 {address} 断开连接')
    
    def process_received_data(self, data, client_socket):
        """处理接收到的数据"""
        try:
            # 这里实现您的消息处理逻辑
            message = data.decode('utf-8')
            self.get_logger().info(f'收到消息: {message}')
            
            # 示例：简单回显
            response = f"ECHO: {message}"
            client_socket.send(response.encode('utf-8'))
            
        except Exception as e:
            self.get_logger().error(f'处理数据错误: {e}')

def main():
    rclpy.init()
    node = ROSTCPEndpoint()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()