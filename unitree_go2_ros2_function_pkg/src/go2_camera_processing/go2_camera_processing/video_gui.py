#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageViewer(Node):
    def __init__(self):
        super().__init__('video_gui')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image/raw',  
            self.image_callback,
            10
        )
        self.subscription  # 防止未使用变量的警告

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 显示图像
        cv2.imshow("Image Viewer", cv_image)
        cv2.waitKey(1)  # 等待1毫秒以更新窗口

def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # 关闭所有 OpenCV 窗口

if __name__ == '__main__':
       main()