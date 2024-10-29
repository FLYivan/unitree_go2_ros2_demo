# GUI显示机器人信息

"""
高层：
机器人定位
路径规划结果
传感器状态信息

底层：
电池状态
IMU状态

"""

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
import rclpy
from rclpy.node import Node

class DisplayNode(Node):
    def __init__(self):
        super().__init__('display_node')
        # 订阅所需的主题并更新 GUI

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Robot Display')
        # 初始化 GUI 组件

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    display_node = DisplayNode()
    rclpy.spin(display_node)
    display_node.destroy_node()
    rclpy.shutdown()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()