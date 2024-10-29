import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.publisher_ = self.create_publisher(Path, 'planned_path', 10)
        self.algorithm_subscriber = self.create_subscription(
            String,
            'navigation_algorithm',
            self.algorithm_callback,
            10)
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        
        self.current_algorithm = 'A*'  # 默认使用A*算法
        self.map_data = None  # 用于存储地图数据
        self.timer = self.create_timer(1.0, self.plan_path)
        self.get_logger().info(f'Using {self.current_algorithm} algorithm for path planning.')

    def algorithm_callback(self, msg):
        # 更新当前使用的算法
        self.current_algorithm = msg.data
        self.get_logger().info(f'Switched to {self.current_algorithm} algorithm.')

    def map_callback(self, msg):
        # 存储接收到的地图数据
        self.map_data = msg
        self.get_logger().info('Map data received.')

    def plan_path(self):
        if self.map_data is None:
            self.get_logger().warn('No map data available. Cannot plan path.')
            return

        path = Path()
        # 根据当前选择的算法进行路径规划
        if self.current_algorithm == 'A*':
            path = self.plan_with_a_star()
        elif self.current_algorithm == 'Dijkstra':
            path = self.plan_with_dijkstra()
        elif self.current_algorithm == 'RRT':
            path = self.plan_with_rrt()
        else:
            self.get_logger().warn(f'Unknown algorithm: {self.current_algorithm}. Using default A*.')
            path = self.plan_with_a_star()

        self.publisher_.publish(path)

    def plan_with_a_star(self):
        # 实现A*算法的路径规划
        path = Path()
        # 使用self.map_data进行路径计算
        return path

    def plan_with_dijkstra(self):
        # 实现Dijkstra算法的路径规划
        path = Path()
        # 使用self.map_data进行路径计算
        return path

    def plan_with_rrt(self):
        # 实现RRT算法的路径规划
        path = Path()
        # 使用self.map_data进行路径计算
        return path

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()