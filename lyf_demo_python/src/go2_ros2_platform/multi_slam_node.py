import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from abc import ABC, abstractmethod

# 定义 SLAM 抽象基类
class SLAMBase(ABC):
    def __init__(self):
        self.current_position = (0, 0)  # 机器人当前位置
        self.map_data = []  # 存储地图数据

    @abstractmethod
    def process_data(self, data):
        """处理传感器数据并更新地图"""
        pass

    @abstractmethod
    def get_map(self):
        """返回当前地图"""
        pass

# 实现 RTAB-Map SLAM 算法
class RTABMapSLAM(SLAMBase):
    def process_data(self, data):
        # 处理 RGB-D 数据并更新地图
        print("Processing data with RTAB-Map SLAM")
        # 具体的处理逻辑
        # 例如，更新地图数据
        self.map_data.append(data)

    def get_map(self):
        return "RTAB-Map generated map"

# 实现 Cartographer SLAM 算法
class CartographerSLAM(SLAMBase):
    def process_data(self, data):
        # 处理激光雷达数据并更新地图
        print("Processing data with Cartographer SLAM")
        # 具体的处理逻辑
        # 例如，更新地图数据
        self.map_data.append(data)

    def get_map(self):
        return "Cartographer generated map"

# SLAM 管理器
class SLAMManager:
    def __init__(self):
        self.slam_algorithm = None  # 当前使用的 SLAM 算法

    def set_slam_algorithm(self, slam_algorithm: SLAMBase):
        """设置当前使用的 SLAM 算法"""
        self.slam_algorithm = slam_algorithm

    def process_data(self, data):
        """处理传感器数据"""
        if self.slam_algorithm:
            self.slam_algorithm.process_data(data)  # 调用当前 SLAM 算法的处理方法
        else:
            print("No SLAM algorithm set!")  # 如果没有设置 SLAM 算法，输出警告

    def get_map(self):
        """返回当前地图"""
        if self.slam_algorithm:
            return self.slam_algorithm.get_map()  # 调用当前 SLAM 算法的获取地图方法
        return None  # 如果没有设置 SLAM 算法，返回 None

# SLAM 节点
class SLAMNode(Node):
    def __init__(self):
        super().__init__('slam_node')
        self.get_logger().info("SLAM Node started")
        
        self.slam_manager = SLAMManager()
        
        # 默认使用 RTAB-Map SLAM
        self.slam_manager.set_slam_algorithm(RTABMapSLAM())

        # 订阅运动控制节点的消息
        self.create_subscription(String, 'exploration_command', self.exploration_callback, 10)
        # 订阅算法切换命令
        self.create_subscription(String, 'slam_algorithm_switch', self.switch_algorithm_callback, 10)

    def exploration_callback(self, msg):
        self.get_logger().info(f"Received exploration command: {msg.data}")
        self.start_slam()

    def start_slam(self):
        self.get_logger().info("Starting SLAM processing...")
        sample_data = "sensor_data"  # 这里可以是实际的传感器数据
        self.slam_manager.process_data(sample_data)

    def switch_algorithm_callback(self, msg):
        """处理算法切换命令"""
        algorithm_name = msg.data
        self.switch_slam_algorithm(algorithm_name)

    def switch_slam_algorithm(self, algorithm_name):
        if algorithm_name == "rtabmap":
            self.slam_manager.set_slam_algorithm(RTABMapSLAM())
            self.get_logger().info("Switched to RTAB-Map SLAM")
        elif algorithm_name == "cartographer":
            self.slam_manager.set_slam_algorithm(CartographerSLAM())
            self.get_logger().info("Switched to Cartographer SLAM")
        else:
            self.get_logger().warn("Unknown SLAM algorithm")

def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAMNode()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()