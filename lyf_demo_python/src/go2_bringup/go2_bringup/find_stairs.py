import rclpy

from rclpy.node import Node
from unitree_go.msg import SportModeState                           # 导入SportModeState消息类型，订阅状态信息
from unitree_api.msg import Request                                 # 导入Request消息类型，发送运动指令

TOPIC = "/utlidar/height_map_array"

class FindStairs(Node):
    def __init__(self):
        super().__init__('find_stairs')

        self.sub = self.create_subscription(SportModeState,TOPIC,self.high_state_callback,10)

    def high_state_callback(self,msg):
        highdata = msg.data
        self.get_logger().info(f'高度数据为{highdata}')


def main(args=None):
    rclpy.init(args=args)
    node = FindStairs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown
    
if __name__ == '__main__':
    main()