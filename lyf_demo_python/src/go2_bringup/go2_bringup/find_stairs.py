import rclpy

from rclpy.node import Node
from unitree_go.msg import HeightMap                           # 导入SportModeState消息类型，订阅状态信息


TOPIC = "/utlidar/height_map_array"

class FindStairs(Node):
    def __init__(self):
        super().__init__('find_stairs')

        self.sub = self.create_subscription(HeightMap,TOPIC,self.high_state_callback,10)

    def high_state_callback(self,msg):
        highdata = msg.data
        for index, value in enumerate(highdata):
            if value != 1000000000.0:
                row, col = divmod(index,self.get_matrix_width(highdata))
                self.get_logger().info(f'数据为{value},行：{row}，列：{col}')
        

    def get_matrix_width(self,data):
        return 128


def main(args=None):
    rclpy.init(args=args)
    node = FindStairs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown
    
if __name__ == '__main__':
    main()
