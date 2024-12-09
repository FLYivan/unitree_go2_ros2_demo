#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt

class StairDetector(Node):
    def __init__(self):
        super().__init__('stair_detector')
        
        # 参数设置
        self.params = {
            'height_threshold': 0.15,    # 高度变化阈值（米）
            'max_height_diff': 0.20,     # 最大台阶高度差（米）
            'min_step_depth': 0.25,      # 最小台阶深度（米）
            'max_step_depth': 0.35,      # 最大台阶深度（米）
            'min_step_width': 0.60,      # 最小台阶宽度（米）
            'invalid_value': 1000000000.0,  # 无效值
            'min_steps': 2               # 最小台阶数
        }

        # 创建订阅者
        self.height_map_sub = self.create_subscription(
            OccupancyGrid,
            '/height_map',  # 根据实际话题名修改
            self.height_map_callback,
            10
        )

        # 创建发布者（用于可视化）
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'detected_stairs',
            10
        )

        # 创建定时器用于定期处理
        self.timer = self.create_timer(1.0, self.process_timer_callback)

        # 存储最新的高度图数据
        self.latest_height_data = None
        self.map_metadata = None

        self.get_logger().info('Stair detector node initialized')

    def height_map_callback(self, msg):
        """处理接收到的高度图数据"""
        try:
            # 假设消息中包含地图尺寸信息
            width = msg.layout.dim[0].size
            height = msg.layout.dim[1].size
            
            # 将一维数组转换为二维数组
            height_data = np.array(msg.data).reshape(height, width)
            self.latest_height_data = height_data
            
            # 存储分辨率信息（如果可用）
            self.resolution = msg.layout.dim[0].stride  # 假设分辨率信息在这里
            
            self.get_logger().debug('Received height map data')
        except Exception as e:
            self.get_logger().error(f'Error processing height map: {str(e)}')

    def process_timer_callback(self):
        """定期处理高度图数据"""
        if self.latest_height_data is None:
            return

        try:
            # 检测楼梯
            stairs = self.detect_stairs(self.latest_height_data)
            
            # 发布可视化标记
            self.publish_visualization_markers(stairs)
            
            # 打印检测结果
            self.get_logger().info(
                f'Detected {len(stairs["up_stairs"])} up stairs and '
                f'{len(stairs["down_stairs"])} down stairs'
            )
        except Exception as e:
            self.get_logger().error(f'Error in processing: {str(e)}')

    def detect_stairs(self, height_data):
        """检测楼梯的主要函数"""
        # 处理高度数据
        processed_data = self.process_height_data(height_data)
        
        # 检测高度变化
        height_changes = self.detect_height_changes(processed_data)
        
        # 分析楼梯模式
        stairs = self.analyze_stair_pattern(height_changes)
        
        return stairs

    def process_height_data(self, height_data):
        """
        处理高度图数据，填充无效值
        - 遍历高度图的每个像素
        - 对于每个无效值位置，获取其有效邻居的值
        - 用邻居的平均值填充当前无效位置
        """
        processed_data = height_data.copy()  # 复制输入的高度数据
        mask = processed_data == self.params['invalid_value']  # 创建无效值的掩码，numpy常用语法，等于掩码的值为True，否则为False
        
        # 使用邻近有效值填充
        for i in range(processed_data.shape[0]):  # 遍历数据的行
            for j in range(processed_data.shape[1]):  # 遍历数据的列
                if mask[i,j]:  # 如果当前点是无效值
                    valid_neighbors = self.get_valid_neighbors(processed_data, i, j)  # 获取有效的相邻点值
                    if valid_neighbors:  # 如果存在有效的相邻点
                        processed_data[i,j] = np.mean(valid_neighbors)  # 用相邻点的平均值填充当前点
        
        return processed_data  # 返回处理后的数据

    def get_valid_neighbors(self, data, i, j):
        """获取有效的相邻点值"""
        valid_values = []  # 初始化存储有效值的列表
        for di in [-1, 0, 1]:  # 遍历相邻点的行偏移
            for dj in [-1, 0, 1]:  # 遍历相邻点的列偏移
                if (0 <= i+di < data.shape[0] and  # 检查行索引是否在有效范围内
                    0 <= j+dj < data.shape[1] and  # 检查列索引是否在有效范围内
                    data[i+di,j+dj] != self.params['invalid_value']):  # 检查相邻点是否为有效值
                    valid_values.append(data[i+di,j+dj])  # 将有效的相邻点值添加到列表中
        return valid_values  # 返回所有有效的相邻点值列表

    # def detect_height_changes(self, height_data):
    #     """检测高度变化"""
    #     height_changes = []  # 初始化存储高度变化的列表
    #     rows, cols = height_data.shape  # 获取高度数据的行数和列数
        
    #     for i in range(rows-1):  # 遍历行（除最后一行），因为需要和下一行比较
    #         for j in range(cols):  # 遍历列
    #             if (height_data[i][j] != self.params['invalid_value'] and  # 检查当前点是否有效
    #                 height_data[i+1][j] != self.params['invalid_value']):  # 检查下一行的点是否有效
                    
    #                 height_diff = height_data[i+1][j] - height_data[i][j]  # 计算高度差
                    
    #                 if abs(height_diff) >= self.params['height_threshold']:  # 判断高度差是否超过阈值，过滤掉微小的高度变化
    #                     # 确定楼梯类型（上楼或下楼）
    #                     if height_diff < 0:
    #                         stair_type = 'down'
    #                     else:
    #                         stair_type = 'up'   
    #                     height_changes.append({  # 将检测到的台阶信息添加到列表中
    #                         'position': (i, j),  # 记录位置坐标
    #                         'diff': height_diff,  # 记录高度差值
    #                         'type': stair_type  # 记录台阶类型
    #                     })
        
    #     return height_changes  # 返回检测到的所有高度变化
    
    def detect_height_changes(self, height_data):
        """多方向梯度检测"""
        height_changes = []  # 初始化存储高度变化的列表
        rows, cols = height_data.shape  # 获取高度数据的行数和列数
        
        # 定义8个方向的偏移量（上、下、左、右、四个对角线）
        directions = [
            (-1,0), (1,0), (0,-1), (0,1),  # 上下左右四个方向
            (-1,-1), (-1,1), (1,-1), (1,1)  # 四个对角线方向
        ]
        
        for i in range(1, rows-1):  # 遍历行(排除边界)
            for j in range(1, cols-1):  # 遍历列(排除边界)
                if height_data[i][j] == self.params['invalid_value']:  # 如果当前点是无效值
                    continue  # 跳过此点
                    
                # 检查所有8个方向
                for di, dj in directions:  # 遍历8个方向
                    ni, nj = i + di, j + dj  # 计算邻居点的坐标
                    if (0 <= ni < rows and 0 <= nj < cols and   # 检查邻居点是否在有效范围内
                        height_data[ni][nj] != self.params['invalid_value']):  # 检查邻居点是否有效
                        
                        height_diff = height_data[ni][nj] - height_data[i][j]  # 计算高度差
                        
                        if abs(height_diff) >= self.params['height_threshold']:  # 判断高度差是否超过阈值
                            # 计算梯度方向（可用于确定楼梯朝向）
                            gradient_angle = np.arctan2(di, dj)  # 计算梯度角度
                            height_changes.append({  # 添加检测到的台阶信息
                                'position': (i, j),  # 记录位置坐标
                                'diff': height_diff,  # 记录高度差
                                'type': 'up' if height_diff > 0 else 'down',  # 记录台阶类型
                                'direction': gradient_angle  # 记录梯度方向
                            })
        
        return height_changes  # 返回检测到的所有高度变化

    def analyze_stair_pattern(self, height_changes):
        """分析楼梯模式"""
        if not height_changes:
            return {'up_stairs': [], 'down_stairs': []}

        # 首先按上下楼梯分类
        up_candidates = [c for c in height_changes if c['type'] == 'up']
        down_candidates = [c for c in height_changes if c['type'] == 'down']
        
        # 对每种类型进行方向聚类
        up_stairs = self.cluster_and_verify_stairs(up_candidates)
        down_stairs = self.cluster_and_verify_stairs(down_candidates)
        
        return {
            'up_stairs': up_stairs,
            'down_stairs': down_stairs
        }

    def cluster_and_verify_stairs(self, candidates):
        """按方向聚类并验证楼梯模式"""
        if len(candidates) < self.params['min_steps']:
            return []
            
        # 方向聚类
        direction_clusters = {}
        angle_threshold = np.pi / 8  # 22.5度的容差
        
        for candidate in candidates:
            angle = candidate['direction']
            # 将角度规范化到主方向
            normalized_angle = round(angle / angle_threshold) * angle_threshold
            
            if normalized_angle not in direction_clusters:
                direction_clusters[normalized_angle] = []
            direction_clusters[normalized_angle].append(candidate)
        
        verified_stairs = []
        # 对每个方向簇进行验证
        for angle, cluster in direction_clusters.items():
            if len(cluster) >= self.params['min_steps']:
                verified = self.verify_stair_pattern(cluster, angle)
                verified_stairs.extend(verified)
        
        return verified_stairs

    def verify_stair_pattern(self, stair_candidates, main_direction):
        """验证楼梯模式"""
        verified_stairs = []
        
        # 将候选点投影到主方向上进行排序
        # 使用主方向的单位向量进行投影
        direction_vector = np.array([np.cos(main_direction), np.sin(main_direction)])
        
        # 计算每个点在主方向上的投影距离
        projected_positions = []
        for candidate in stair_candidates:
            pos = np.array(candidate['position'])
            projection = np.dot(pos, direction_vector)
            projected_positions.append((projection, candidate))
        
        # 按投影距离排序
        sorted_candidates = [c for _, c in sorted(projected_positions)]
        
        current_stair = [sorted_candidates[0]]
        
        for i in range(1, len(sorted_candidates)):
            curr_pos = np.array(sorted_candidates[i]['position'])
            prev_pos = np.array(current_stair[-1]['position'])
            
            # 计算实际距离
            distance = np.linalg.norm(curr_pos - prev_pos)
            
            # 计算在主方向上的距离
            proj_distance = abs(np.dot(curr_pos - prev_pos, direction_vector))
            
            # 验证台阶间距
            if (self.params['min_step_depth'] <= proj_distance <= 
                self.params['max_step_depth']):
                # 验证高度差的一致性
                curr_diff = sorted_candidates[i]['diff']
                prev_diff = current_stair[-1]['diff']
                if abs(abs(curr_diff) - abs(prev_diff)) < 0.05:  # 允许5cm的误差
                    current_stair.append(sorted_candidates[i])
            else:
                if len(current_stair) >= self.params['min_steps']:
                    verified_stairs.extend(current_stair)
                current_stair = [sorted_candidates[i]]
        
        # 处理最后一组
        if len(current_stair) >= self.params['min_steps']:
            verified_stairs.extend(current_stair)
        
        return verified_stairs

    def publish_visualization_markers(self, stairs):
        """发布可视化标记"""
        marker_array = MarkerArray()
        marker_id = 0
        
        # 为上楼梯创建标记
        for stair in stairs['up_stairs']:
            marker = self.create_stair_marker(stair, marker_id, 'up')
            marker_array.markers.append(marker)
            marker_id += 1
            
        # 为下楼梯创建标记
        for stair in stairs['down_stairs']:
            marker = self.create_stair_marker(stair, marker_id, 'down')
            marker_array.markers.append(marker)
            marker_id += 1
            
        self.marker_pub.publish(marker_array)

    def create_stair_marker(self, stair, marker_id, stair_type):
        """创建楼梯可视化标记"""
        marker = Marker()
        marker.header.frame_id = "map"  # 使用适当的框架ID
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # 设置标记位置
        pos = stair['position']
        marker.pose.position.x = pos[1] * self.map_metadata.resolution
        marker.pose.position.y = pos[0] * self.map_metadata.resolution
        marker.pose.position.z = 0.0
        
        # 设置标记大小
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # 设置标记颜色
        marker.color = ColorRGBA()
        if stair_type == 'up':
            marker.color.r = 1.0
            marker.color.g = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    stair_detector = StairDetector()
    
    try:
        rclpy.spin(stair_detector)
    except KeyboardInterrupt:
        pass
    finally:
        stair_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()