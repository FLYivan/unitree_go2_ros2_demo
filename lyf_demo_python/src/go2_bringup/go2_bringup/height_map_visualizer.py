#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from unitree_go.msg import HeightMap
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import colorsys

from builtin_interfaces.msg import Duration  

HIGHTMAPTOPIC = "/utlidar/height_map_array"

class HeightMapVisualizer(Node):
    def __init__(self):
        super().__init__('height_map_visualizer')
        
        # 创建订阅者（订阅输入的高度图数据）
        self.height_sub = self.create_subscription(
            HeightMap,
            HIGHTMAPTOPIC,  # 输入的高度图话题名
            self.height_callback,
            10
        )
        
        # 创建发布者（用于Rviz2显示）
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'height_markers',  # 输出的可视化话题名
            10
        )
        
        # 存储最新的高度数据
        self.latest_height_data = None
        self.map_metadata = None
        
        self.get_logger().info('Height map visualizer initialized')
    
    def height_callback(self, msg):
        """处理接收到的高度图数据"""
        try:
            # 保存地图元数据
            self.map_metadata = msg
            
            # 将一维数组转换为二维数组
            height_data = np.array(msg.data).reshape(
                msg.height, msg.width)
            
            # 打印原始数组的基本统计信息
            self.get_logger().info(f'原始数组数据 - min: {np.min(height_data)}, '
                                f'max: {np.max(height_data)}, '
                                f'mean: {np.mean(height_data):.2f}')

            # 处理高度数据
            self.latest_height_data = self.process_height_data(height_data)

            # 打印第一步处理的数组的基本统计信息
            self.get_logger().info(f'第1步处理后数组数据 - min: {np.min(self.latest_height_data)}, '
                                f'max: {np.max(self.latest_height_data)}, '
                                f'mean: {np.mean(self.latest_height_data):.2f}')
            
            # 立即发布处理后的数据
            self.publish_height_markers()


            
            self.get_logger().debug('Processed and published new height markers')
            
        except Exception as e:
            self.get_logger().error(f'Error processing height map: {str(e)}')
    
    def process_height_data(self, height_data):
        """处理高度数据（根据需要调整）"""
        # 处理无效值
        invalid_mask = height_data == 1000000000.0  # 假设这是无效值
        processed_data = height_data.copy()
        processed_data[invalid_mask] = -1.0  # 将无效值设为-1
        
        # 对有效数据进行处理（非-1的数据）
        valid_mask = processed_data != -1.0
        processed_data[valid_mask] = np.clip(processed_data[valid_mask], 0.0, 100.0)
        
        return processed_data
    
    def publish_height_markers(self):
        """发布高度地图的3D标记用于可视化"""
        if self.latest_height_data is None or self.map_metadata is None:
            return
            
        marker_array = MarkerArray()
        
        # 创建一个立方体标记
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = self.map_metadata.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "height_map"                                # 命名空间
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # 设置marker为永久显示
        marker.lifetime = Duration()  # 默认为0，表示永久显示

        # 设置标记的全局属性
        marker.scale.x = self.map_metadata.resolution
        marker.scale.y = self.map_metadata.resolution
        marker.scale.z = self.map_metadata.resolution                # 高度缩放因子
        

        # 遍历高度图的每个点
        for i in range(self.map_metadata.height):
            for j in range(self.map_metadata.width):
                height = self.latest_height_data[i, j]
                if height >= 0:  # 只显示非零高度
                    # 计算世界坐标
                    x = self.map_metadata.origin[0] + j * self.map_metadata.resolution
                    y = self.map_metadata.origin[1] + i * self.map_metadata.resolution
                    z = height / 2.0  # 将立方体放在正确的高度
                    
                    # 添加点
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = z
                    marker.points.append(point)
                    
                    # 根据高度设置颜色（从蓝到红的渐变）
                    color = self.height_to_color(height)
                    marker.colors.append(color)
      
        
        marker_array.markers.append(marker)
        self.marker_publisher.publish(marker_array)
    
    def height_to_color(self, height):
        """将高度值转换为颜色，使用HSV颜色空间实现更丰富的渐变效果"""

        # 将高度值映射到HSV颜色空间
        # hue: 240 (蓝) -> 0 (红)
        # saturation: 保持在1.0
        # value: 在0.7-1.0之间变化，增加亮度区分
        hue = (1.0 - height) * 0.66  # 0.66 约等于 240/360，将蓝色映射到色相值
        saturation = 1.0
        value = 0.7 + (height * 0.3)  # 高度越高，亮度越大

        # 将HSV转换为RGB
        rgb = colorsys.hsv_to_rgb(hue, saturation, value)
        
        color = ColorRGBA()
        color.r = rgb[0]
        color.g = rgb[1]
        color.b = rgb[2]
        color.a = 0.9  # 略微提高不透明度以增加可见度
        
        return color
    

        """色弱友好方案"""

        # # 使用黄色 (255, 255, 0) 到蓝色 (0, 0, 255) 的线性插值
        # # 低高度为蓝色，高高度为黄色
        # color = ColorRGBA()
        # color.r = float(height)  # 红色分量随高度增加
        # color.g = float(height)  # 绿色分量随高度增加
        # color.b = 1.0 - float(height)  # 蓝色分量随高度减少
        # color.a = 0.9  # 保持不透明度

        # return color
    

def main(args=None):
    rclpy.init(args=args)
    visualizer = HeightMapVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()