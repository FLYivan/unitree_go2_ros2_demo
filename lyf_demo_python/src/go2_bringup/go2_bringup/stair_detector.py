#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String  # 用于发布检测结果

from unitree_go.msg import HeightMap         

HIGHTMAPTOPIC = "/utlidar/height_map_array"

class StairDetector(Node):
    def __init__(self):
        super().__init__('stair_detector')
        
        # 参数设置
        self.params = {
            'height_threshold': 0.03,     # 台阶高度阈值（米），小于这个阈值的不认为是台阶（国标楼梯高度0.15米）
            'scan_distance': 2.5,        # 前向扫描距离（米）
            'scan_width': 3.0,           # 扫描宽度（米）
            'min_steps': 2,              # 最小台阶数
            'invalid_value': 1000000000.0,  # 无效值标记
            'max_approach_angle': 15.0,  # 最大允许的接近角度（度）
            'min_row_gap': 3,            # 每个台阶边缘到下个台阶边沿至少要延伸的水平栅格数（考虑正常台阶有一定深度）
            'height_tolerance': 0.03,     # 相邻栅格点的允许高度差的误差
            'min_points_per_step': 3     # 判断朝向夹角时，需要存在连续高度差突变的栅格点数量
        }

        # 创建订阅者
        self.height_map_sub = self.create_subscription(
            HeightMap,
            HIGHTMAPTOPIC,
            self.height_map_callback,
            10
        )

        # 创建发布者（用于发布检测结果）
        self.result_pub = self.create_publisher(
            String,
            'stairs_detection_result',
            10
        )

        self.map_metadata = None  # 初始化map_metadata

        self.get_logger().info('Stair detector node initialized')

    def detect_height_change(self, height_data):
        # 首先检查是否已经收到地图元数据
        if self.map_metadata is None:
            self.get_logger().warn('No map metadata received yet')
            return "NO_STAIRS"

        """简化的楼梯检测函数"""
        rows, cols = height_data.shape
        resolution = self.map_metadata.resolution
        
        # 找到机器人位置（假设在地图中心）
        robot_row = rows // 2
        robot_col = cols // 2
        
        # 计算扫描范围（像素单位）
        forward_pixels = int(self.params['scan_distance'] / resolution)
        width_pixels = int(self.params['scan_width'] / resolution)
        
        # 定义扫描区域（机器人前半部分区域）
        scan_start_row = robot_row - forward_pixels
        scan_end_row = robot_row
        scan_start_col = robot_col - width_pixels // 2
        scan_end_col = robot_col + width_pixels // 2
        
        # 确保范围在地图内
        scan_start_row = max(1, scan_start_row)  # 从1开始以便检查时第0行不会越界
        scan_end_row = min(rows, scan_end_row)
        scan_start_col = max(0, scan_start_col)
        scan_end_col = min(cols, scan_end_col)

        # 存储高度变化点
        height_changes = []
        for i in range(scan_start_row, scan_end_row):
            for j in range(scan_start_col, scan_end_col):
                if (height_data[i][j] != self.params['invalid_value'] and 
                    height_data[i-1][j] != self.params['invalid_value']):
                    
                    height_diff = height_data[i-1][j] - height_data[i][j]
                    
                    if abs(height_diff) >= self.params['height_threshold']:
                        # 检查8连通邻域
                        similar_positions = []  # 存储所有相似点的位置
                        checked_positions = set()  # 创建一个记录已检查的位置的集合

                        # 8个方向的偏移量（不包括中心点）
                        directions = [(-1,-1), (-1,0), (-1,1),
                                   (0,-1),          (0,1),
                                   (1,-1),  (1,0),  (1,1)]

                        for di, dj in directions:
                            new_i, new_j = i + di, j + dj
                            pos = (di, dj)
                            
                            # 检查边界条件和是否已检查
                            if (scan_start_col <= new_j < scan_end_col and 
                                scan_start_row <= new_i < scan_end_row and
                                pos not in checked_positions):
                                
                                # 计算该点的高度差
                                if (height_data[new_i][new_j] != self.params['invalid_value'] and 
                                    height_data[new_i-1][new_j] != self.params['invalid_value']):
                                    neighbor_height_diff = height_data[new_i][new_j] - height_data[new_i-1][new_j]
                                    
                                    # 检查高度差是否相似
                                    if abs(neighbor_height_diff - height_diff) < self.params['height_tolerance']:
                                        similar_positions.append((di, dj))
                                
                                checked_positions.add(pos)                      # 向已检查位置点集合中添加元素

                        # 检查是否存在至少2个不相邻的相似点
                        non_adjacent_count = 0
                        for idx1, pos1 in enumerate(similar_positions):
                            is_isolated = True
                            for idx2, pos2 in enumerate(similar_positions):
                                if idx1 != idx2:                                        #  idx1和idx2都是索引值，是为了避免不是比较同一个点
                                    # 检查两点是否相邻（曼哈顿距离大于1即为不相邻）
                                    if abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1]) <= 1:
                                        is_isolated = False
                                        break
                            if is_isolated:
                                non_adjacent_count += 1

                        # 如果存在至少2个不相邻的相似点，则认为这个点是台阶边沿，而不是噪声
                        if non_adjacent_count >= 2:
                            height_changes.append({
                                'row': i,
                                'col': j,
                                'height_diff': height_diff
                            })
                        
        return height_changes
    

    # 楼梯检测
    def detect_stairs(self, data):

        # 是否存在高度变化的点
        if not height_changes:
            return "NO_STAIRS_POINT"

        # 按行位置排序高度变化点
        height_changes.sort(key=lambda x: x['row'], reverse=True)  # 按row从大到小排序
        
        # 统计实际台阶数
        up_stairs = 0
        down_stairs = 0
        height_changes.sort(key=lambda x: x['row'], reverse=True)  # 从大到小排序，最近的点在前面
        
        last_stair_row = self.params['scan_start_row']  # 从机器人所在位置开始检测
        
        for change in height_changes:
            if last_stair_row - change['row'] >= self.params['min_row_gap']:
                if change['height_diff'] > 0:  # height_diff = height_data[i-1][j] - height_data[i][j]
                    up_stairs += 1             # 远处比近处高，是上楼梯
                elif change['height_diff'] < 0:
                    down_stairs += 1           # 远处比近处低，是下楼梯
                last_stair_row = change['row']


        # 添加台阶方向检测
        if up_stairs >= self.params['min_steps'] or down_stairs >= self.params['min_steps']:            # 如果有至少X个突变点
            # 计算台阶的方向角度
            angle = self.calculate_stairs_angle(height_changes)
            
            if abs(angle) >= self.params['max_approach_angle']:  # 比如 15度
                return "ADJUST_POSE"  # 需要调整姿态
            else:
                # 分别判断上下楼梯的情况
                if up_stairs >= self.params['min_steps']:
                    return "UP"
                elif down_stairs >= self.params['min_steps']:
                    return "DOWN"
                else:
                    return "NO_STAIRS"  # 既不满足上楼梯也不满足下楼梯的条件
                

    def pose_adjust(self, height_changes):

        angle = self.calculate_stairs_angle(height_changes)

        if abs(angle) >= self.params['max_approach_angle']:  # 比如 15度
                return "ADJUST_POSE"  # 需要调整姿态
        else:
            # 将高度变化点转化为实际的楼梯台阶检测
            pass
            # 根据楼梯方向，返回是调整为上楼梯姿态，还是下楼梯姿态
            # # 分别判断上下楼梯的情况
            # if up_stairs >= self.params['min_steps']:
            #     return "UP"
            # elif down_stairs >= self.params['min_steps']:
            #     return "DOWN"
            # else:
            #     return "NO_STAIRS"  # 既不满足上楼梯也不满足下楼梯的条件
        
    def calculate_stairs_angle(self, height_changes):
        """计算前方障碍物边缘线与机器人朝向的夹角
        返回值: 角度（度）
        - 正值：需要向左转（如边缘线是 / 时）
        - 负值：需要向右转（如边缘线是 \ 时）
        - 0度: 表示台阶边缘线与机器人垂直对齐
        """
        # 收集最近的一排高度突变点
        edge_points = []

        # 初始化 min_row 为第一个元素的 row 值（或一个足够大的数）
        min_row = float('inf')  # 初始化为无穷大
        # 遍历所有的 height_changes
        for change in height_changes:
            if change['row'] < min_row:
                min_row = change['row']

        search_range = 3  # 在最近点3个栅格行范围内的其他高度突变点都考虑

        # 收集在搜索范围内的所有点
        for change in height_changes:
            if change['row'] - min_row <= search_range:  # 修改判断条件
                edge_points.append((change['col'], change['row']))

        # 如果点数太少，返回0表示无法判断角度
        if len(edge_points) < self.params['min_points_per_step']:
            return 0.0

        # 使用numpy的最小二乘法拟合直线
        x = []
        y = []
        for p in edge_points:
            x.append(p[1])  # col值作为x
            y.append(p[0])  # row值作为y
        x = np.array(x)
        y = np.array(y)
        
        # 拟合直线 y = kx + b
        A = np.vstack([x, np.ones(len(x))]).T
        k, b = np.linalg.lstsq(A, y, rcond=None)[0]

        # 计算边缘线与Y轴的夹角
        angle = np.degrees(np.arctan(k))
        
        # 直接返回角度：
        # - 当边缘线是 / 时，angle > 0，需要向左转angle角度，以垂直面向楼梯
        # - 当边缘线是 \ 时，angle < 0，需要向右转angle角度，以垂直面向楼梯
        return angle


    def height_map_callback(self, msg):
        """处理高度图数据"""
        try:
            # 将一维数组转换为二维数组
            height_data = np.array(msg.data).reshape(msg.height, msg.width)
            self.map_metadata = msg
            
            # 检测高度变化点
            height_change_point = self.detect_height_change(height_data)

            # 根据角度调在机器人姿态
            self.pose_adjust(height_change_point)


            # 检测楼梯
            result = self.detect_stairs(height_data)
            
            # # 发布检测结果
            # result_msg = String()
            # result_msg.data = result
            # self.result_pub.publish(result_msg)
            
            
            # 记录检测结果
            self.get_logger().info(f'Stair detection result: {result}')
            
            # 根据结果采取行动
            if result == "ADJUST_POSE":
                self.get_logger().info('Need to adjust robot pose before climbing')
                # 这里可以发布姿态调整命令
                # self.pose_adjustment_pub.publish(...)
            elif result in ["UP", "DOWN"]:
                self.get_logger().info(f'Ready to climb {result}')
                # 发布爬楼梯命令
                # self.climb_command_pub.publish(...)
            
        except Exception as e:
            self.get_logger().error(f'Error processing height map: {str(e)}')

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