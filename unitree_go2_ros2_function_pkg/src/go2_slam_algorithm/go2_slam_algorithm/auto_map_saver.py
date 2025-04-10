#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import signal
import sys
import subprocess
from datetime import datetime
import os

class AutoMapSaver(Node):
    def __init__(self, name):
        super().__init__(name)
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 获取当前节点所在目录的路径
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # 设置地图保存路径为同级的 maps 目录
        self.map_save_dir = os.path.join(os.path.dirname(current_dir), 'maps')
        
        # 如果 maps 目录不存在，创建它
        if not os.path.exists(self.map_save_dir):
            os.makedirs(self.map_save_dir)
            self.get_logger().info(f'Created maps directory at: {self.map_save_dir}')
        
        self.get_logger().info('Auto map saver node initialized')

    def signal_handler(self, sig, frame):
        """处理关闭信号"""
        self.get_logger().info('Received shutdown signal, saving map...')
        
        try:
            # 生成带时间戳的地图文件名（不包含路径）
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            map_filename = f'map_{timestamp}'  # 这是纯文件名，不包含路径
            
            # 完整的地图保存路径（包含目录路径和文件名）
            map_full_path = os.path.join(self.map_save_dir, map_filename)
            
            # 调用 map_saver_cli 保存地图
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_full_path,
                '--ros-args', '-p', 'save_map_timeout:=10.0'
            ]
            
            # 执行命令
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            if result.returncode == 0:
                self.get_logger().info(f'Map saved successfully to: {map_filename}')
            else:
                self.get_logger().error(f'Failed to save map: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'Error saving map: {str(e)}')
        
        # 清理并关闭节点
        self.get_logger().info('Shutting down...')
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = AutoMapSaver('auto_map_saver')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()