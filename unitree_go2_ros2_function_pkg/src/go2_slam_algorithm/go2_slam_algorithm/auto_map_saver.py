#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal
import sys
import subprocess
from datetime import datetime
import os
import threading
import time

class AutoMapSaver(Node):
    def __init__(self):
        super().__init__('auto_map_saver')
        
        # 初始化信号标志
        self.shutdown_flag = threading.Event()
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 获取当前节点所在目录的路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.map_save_dir = os.path.join(os.path.dirname(current_dir), 'maps')
        
        if not os.path.exists(self.map_save_dir):
            os.makedirs(self.map_save_dir)
            self.get_logger().info(f'Created maps directory at: {self.map_save_dir}')
        
        self.get_logger().info('Auto map saver node initialized')

    def save_map(self):
        """保存地图的具体实现"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            map_filename = f'map_{timestamp}'
            map_full_path = os.path.join(self.map_save_dir, map_filename)
            
            self.get_logger().info('Starting map saving process...')
            
            # 调用 map_saver_cli 保存地图
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_full_path,
                '--ros-args', '-p', 'save_map_timeout:=10.0'
            ]
            
            self.get_logger().info(f'Saving map to: {map_full_path}')
            
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=15  # 设置超时时间
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'Map saved successfully as: {map_filename}')
                self.get_logger().info(f'Map location: {map_full_path}')
                return True
            else:
                self.get_logger().error(f'Failed to save map: {result.stderr}')
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map saving process timed out')
            return False
        except Exception as e:
            self.get_logger().error(f'Error saving map: {str(e)}')
            return False

    def signal_handler(self, sig, frame):
        """处理关闭信号"""
        if self.shutdown_flag.is_set():
            return
        
        self.shutdown_flag.set()
        self.get_logger().info('Received shutdown signal')
        
        # 创建一个新线程来处理地图保存
        save_thread = threading.Thread(target=self.shutdown_procedure)
        save_thread.start()
        
        # 等待保存完成，但最多等待20秒
        save_thread.join(timeout=20)

    def shutdown_procedure(self):
        """关机程序"""
        self.get_logger().info('Starting shutdown procedure...')
        
        # 先保存地图
        if self.save_map():
            self.get_logger().info('Map saved successfully, proceeding with shutdown')
        else:
            self.get_logger().warn('Map saving failed or timed out')
        
        # 确保其他节点有时间完成它们的清理工作
        time.sleep(2)
        
        # 关闭 ROS 节点
        self.get_logger().info('Shutting down ROS node...')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AutoMapSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 确保节点被正确销毁
        if not node.shutdown_flag.is_set():
            node.signal_handler(signal.SIGINT, None)
        node.destroy_node()

if __name__ == '__main__':
    main()