import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation as R

class PoseTransformNode(Node):
    def __init__(self):
        super().__init__('pose_transform_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            'topic_a',
            self.pose_callback,
            10)
        self.initial_position = None
        self.initial_orientation = None

    def pose_callback(self, msg):
        # 获取当前位置信息
        current_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        # 获取当前四元数
        current_orientation = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        if self.initial_position is None and self.initial_orientation is None:
            # 设置初始位置和姿态偏移
            self.initial_position = current_position
            self.initial_orientation = R.from_quat(current_orientation)
            self.get_logger().info(f'Set initial position: {self.initial_position}')
            self.get_logger().info(f'Set initial orientation: {self.initial_orientation.as_quat()}')
            return  # 记录初始位置和姿态后，返回以等待下一个回调

        # 计算相对于初始位置的偏移
        adjusted_position = current_position - self.initial_position
        # 还要考虑上电时刻会站起来，或蹲下来

        # 计算相对于初始姿态的偏移
        current_rot = R.from_quat(current_orientation)
        adjusted_orientation = current_rot * self.initial_orientation.inv()
        adjusted_quat = adjusted_orientation.as_quat()

        # 输出调整后的位置信息和姿态
        self.get_logger().info(f'Adjusted Position: {adjusted_position}')
        self.get_logger().info(f'Adjusted Orientation: {adjusted_quat}')
        
        # 创建并发布 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # 设置位置
        odom_msg.pose.pose.position.x = adjusted_position[0]
        odom_msg.pose.pose.position.y = adjusted_position[1]
        odom_msg.pose.pose.position.z = adjusted_position[2]

        # 设置姿态
        odom_msg.pose.pose.orientation.x = adjusted_quat[0]
        odom_msg.pose.pose.orientation.y = adjusted_quat[1]
        odom_msg.pose.pose.orientation.z = adjusted_quat[2]
        odom_msg.pose.pose.orientation.w = adjusted_quat[3]

        # 发布消息
        self.odom_publisher.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    pose_transform_node = PoseTransformNode()
    rclpy.spin(pose_transform_node)
    pose_transform_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()