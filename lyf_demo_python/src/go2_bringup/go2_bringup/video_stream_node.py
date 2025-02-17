#!/usr/bin/env python3
import cv2  # 导入OpenCV库
import rclpy  # 导入ROS2 Python客户端库
from rclpy.node import Node  # 导入Node类
from sensor_msgs.msg import Image  # 导入Image消息类型
from cv_bridge import CvBridge  # 导入OpenCV与ROS图像转换工具

class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')  # 初始化节点
        
        # 创建图像发布者
        self.publisher_ = self.create_publisher(Image, 'video_stream', 10)
        
        # 创建定时器,每0.1秒执行一次回调函数
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 定义GStreamer管道字符串
        self.gstreamer_str = "udpsrc address=230.1.1.1 port=1720 multicast-iface=<interface_name> ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,width=1280,height=720,format=BGR ! appsink drop=1"
        
        # 初始化视频捕获对象
        self.cap = cv2.VideoCapture(self.gstreamer_str, cv2.CAP_GSTREAMER)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        self.get_logger().info('视频流节点已启动')

    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()  # 读取一帧视频
            if ret:
                # 将OpenCV图像转换为ROS消息并发布
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                msg.header.frame_id = "camera"  # 设置frame_id
                msg.header.stamp = self.get_clock().now().to_msg()  # 设置时间戳
                self.publisher_.publish(msg)
                
                # 显示图像(可选)
                cv2.imshow("Input via Gstreamer", frame)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    self.destroy_node()
                    return

    def __del__(self):
        self.cap.release()  # 释放视频捕获对象
        cv2.destroyAllWindows()  # 关闭所有OpenCV窗口

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2
    
    video_stream_node = VideoStreamNode()  # 创建节点实例
    
    try:
        rclpy.spin(video_stream_node)  # 运行节点
    except KeyboardInterrupt:
        pass
    finally:
        video_stream_node.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':
    main()