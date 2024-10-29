#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                                                # 导入ROS2 Python客户端库
from rclpy.node import Node                                                 # 导入Node类
from rclpy.qos import QoSProfile, QoSReliabilityPolicy,QoSHistoryPolicy     # 导入QoS相关类
from std_msgs.msg import String                                             # 导入标准字符串消息类型
from nav_msgs.msg import Odometry                                           # 导入里程计消息类型
import termios                                                              # 导入终端I/O控制模块
import sys                                                                  # 导入系统特定参数和函数
import tty                                                                  # 导入终端控制模块（按键输入相关）
import math                                                                 # 导入数学函数模块


# 自定义消息类型（用于在ros2框架下的宇数slam算法通信）
from unitree_interfaces.msg import QtCommand,QtEdge,QtNode


# 定义主题名称
COMMANDTOPIC = "qt_command"                              # Qt命令话题
NOTICETOPIC = "qt_notice"                                # Qt通知话题
ODOMTOPIC = "lio_sam_ros2/mapping/re_location_odometry"  # 里程计话题
ADDNODETOPIC = "qt_add_node"                             # 添加节点话题
ADDEDGETOPIC = "qt_add_edge"                             # 添加边话题

class NodeAttribute:
    def __init__(self, nodename : int, nodex :float, nodey : float, nodez : float, nodeyaw : float):
        self.nodename = nodename                          # 导航节点编号
        self.nodex = nodex
        self.nodey = nodey
        self.nodez = nodez
        self.nodeyaw = nodeyaw


class EdgeAttribute:
    def __init__(self, edgename : int, edgestart : int, edgeend : int):
        self.edgename = edgename               # 边编号
        self.edgestart = edgestart             # 边起始节点
        self.edgeend = edgeend                 # 边终止节点

class SlamDemo(Node):
    def __init__(self,name):
        super().__init__(name)                          # 初始化节点
        
        self.index = 0                                  # 初始化索引
        self.current_odom = None                        # 初始化当前里程计数据
        self.nodename = 0                               # 初始化节点编号
        self.nodeattribute_list = []                    # 初始化节点属性列表
        self.edgeattribute_list = []                    # 初始化边属性列表

        # 创建一个QoS原则
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,     # 可靠性策略： 确保消息被传递，如果失败会尝试重传
            history=QoSHistoryPolicy.KEEP_LAST,            # 历史策略： 只保留最新的N条消息
            depth=10                                       # N=10
        )

        # 创建发布者
        self.pub_qt_command = self.create_publisher(QtCommand, COMMANDTOPIC, qos_profile)               # 创建Qt命令发布者
        self.pub_qt_node = self.create_publisher(QtNode, ADDNODETOPIC, qos_profile)                     # 创建Qt节点发布者
        self.pub_qt_edge = self.create_publisher(QtEdge, ADDEDGETOPIC, qos_profile)                     # 创建Qt边发布者

        # 创建订阅者
        self.sub_qt_notice = self.create_subscription(
            String,
            NOTICETOPIC,
            self.qt_notice_handler,
            qos_profile
        )  # 创建Qt通知订阅者

        self.sub_odometry = self.create_subscription(
            Odometry,
            ODOMTOPIC,
            self.odometry_handler,
            qos_profile
        )  # 创建里程计订阅者

        self.print_instructions()                                       # 打印使用说明

        # 创建一个定时器，每0.1秒调用一次key_execute_callback
        self.timer = self.create_timer(0.1, self.key_execute_callback)   



    def print_instructions(self):
        print("***********************  Unitree SLAM Demo ***********************")
        print("------------------         q    w    e         -------------------")
        print("------------------         a    s    d         -------------------")
        print("------------------         z    x    c    v    -------------------")
        print("------------------------------------------------------------------")
        print("------------------ q: Close ROS node           -------------------")
        print("------------------ w: Start mapping            -------------------")
        print("------------------ e: End mapping              -------------------")
        print("------------------ a: Start navigation         -------------------")
        print("------------------ s: Pause navigation         -------------------")
        print("------------------ d: Recover navigation       -------------------")
        print("------------------ z: Relocation and init pose -------------------")
        print("------------------ x: Add node and edge        -------------------")
        print("------------------ c: Save node and edge       -------------------")
        print("------------------ v: Delete all node and edge -------------------")
        print("--------------- Press Ctrl + C to exit the program ---------------")
        print("------------------------------------------------------------------")

    # 里程计数据处理函数
    def odometry_handler(self, msg):
        self.current_odom = msg                                                                     # 更新当前里程计数据

    # Qt通知处理函数
    def qt_notice_handler(self, msg):
        seq = msg.data  # 获取消息数据
        index = int(seq[seq.find("index:") + 6 : seq.find(";", seq.find("index:"))])                # 解析索引
        notice = seq[seq.find("notice:") + 7 : seq.find(";", seq.find("notice:"))]                  # 解析通知内容

        if index <= 10000:                                                                          # 处理命令执行反馈
            feedback = int(seq[seq.find("feedback:") + 9 : seq.find(";", seq.find("feedback:"))])
            if feedback == 0 or feedback == -1:
                self.get_logger().warn(f"指令执行错误，index = {index}.")                              # 打印警告信息
            self.get_logger().info(notice)                                                          # 打印通知信息
        elif index == 10001:                                                                        # 处理导航反馈
            arrive = int(seq[seq.find("arrive:") + 7 : seq.find(";", seq.find("arrive:"))])
            self.get_logger().info(f" 我已经到达导航点 {arrive}. {notice}")                             # 打印到达节点信息

    # 开始建图函数
    def start_mapping(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 3                                                                             # 3是开始建图命令
        msg.attribute = 1                                                                           # 1表示使用XT16激光雷达节点
        self.pub_qt_command.publish(msg)

    # 结束建图函数
    def end_mapping(self):
        self.index += 1
        msg = QtCommand()
        msg.command = 4                                                                             # 4是结束建图命令
        msg.attribute = 0
        msg.seq.data = f"index:{self.index};"
        msg.floor_index.append(0)                                                                   # 楼层号，固定为0
        msg.pcdmap_index.append(0)                                                                  # PCD地图号，固定为0
        self.pub_qt_command.publish(msg)

    # 开始重定位函数
    def start_relocation(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 6                                                                             # 6是开始重定位命令
        msg.attribute = 1                                                                           # 1表示使用XT16激光雷达节点
        self.pub_qt_command.publish(msg)

    # 初始化姿态函数
    def init_pose(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 7                                                                             # 7是姿态初始化指令
        msg.quaternion_x = 0.0
        msg.quaternion_y = 0.0
        msg.quaternion_z = 0.0
        msg.quaternion_w = 1.0
        msg.translation_x = 0.0
        msg.translation_y = 0.0
        msg.translation_z = 0.0
        self.pub_qt_command.publish(msg)

    # 开始导航函数
    def start_navigation(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 8                                             # 8是开始导航命令
        self.pub_qt_command.publish(msg)

    # 默认导航函数实现
    def default_navigation(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = "index:" + str(self.index) + ";"
        msg.command = 10                                            # 10是多节点循环导航（默认）命令
        self.pub_qt_command.publish(msg)

    # 关闭所有节点函数实现
    def close_all_node(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = "index:" + str(self.index) + ";"
        msg.command = 99                                            # 99关闭所有节点命令
        self.pub_qt_command.publish(msg)

    # 添加节点和边函数实现
    def add_nodeand_edge(self):
        nodeTemp = NodeAttribute(0,0.0,0.0,0.0,0.0)

        if self.current_odom is None:
            self.get_logger().warn("当前里程计数据为空，无法添加节点和边。")
            return
    
        # 利用四元数计算偏航角 
        siny_cosp = 2 * (self.current_odom.pose.pose.orientation.w * self.current_odom.pose.pose.orientation.z 
                         + self.current_odom.pose.pose.orientation.x * self.current_odom.pose.pose.orientation.y)       # 分子值 
        cosy_cosp = 1 - 2 * (math.pow(self.current_odom.pose.pose.orientation.y, 2) + math.pow(self.current_odom.pose.pose.orientation.z, 2))  # 分母值 
        self.nodename += 1

        nodeTemp.nodex = self.current_odom.pose.pose.position.x
        nodeTemp.nodey = self.current_odom.pose.pose.position.y
        nodeTemp.nodez = 0
        nodeTemp.nodeyaw = math.atan2(siny_cosp, cosy_cosp)                     # 利用四元数计算偏航角 
        nodeTemp.nodename = self.nodename

        self.nodeattribute_list.append(nodeTemp)

        print("添加的导航点: " + str(self.nodename) + "  X:" + str(nodeTemp.nodex) + "  Y:" + str(nodeTemp.nodey) + "  Z:" + str(nodeTemp.nodez) + "  Yaw:" + str(nodeTemp.nodeyaw))
        if self.nodename >= 2:
            self.add_edge(self.nodename - 1, self.nodename - 1, self.nodename)  # 顺序连接节点

    # 添加边函数实现
    def add_edge(self, edgename, start_node, end_node):
        edgeTemp = EdgeAttribute(0,0,0)

        edgeTemp.edgename = edgename
        edgeTemp.edgestart = start_node
        edgeTemp.edgeend = end_node
        self.edgeattribute_list.append(edgeTemp)
        self.edgeattribute_list.append(edgeTemp)

        print("添加的导航边: " + str(edgename) + "  开始导航点:" + str(start_node) + "  结束导航点:" + str(end_node))
        print("--------------------------------------------------------------------------")

    # 保存节点和边函数实现
    def save_nodeand_edge(self):
        nodeMsg = QtNode()
        edgeMsg = QtEdge()
        if len(self.edgeattribute_list) == 0:
            print("\033[1;31m"
                "拓扑图中的边数量为0。"       # 打印红色警告
                "\033[0m")
            return

        self.index += 1
        nodeMsg.seq.data = "index:" + str(self.index) + ";"
        for i in range(len(self.nodeattribute_list)):
            nodeMsg.node.node_name.append(self.nodeattribute_list[i].nodename)    # 节点名称
            nodeMsg.node.node_position_x.append(self.nodeattribute_list[i].nodex) # 点X坐标信息
            nodeMsg.node.node_position_y.append(self.nodeattribute_list[i].nodey) # 点Y坐标信息
            nodeMsg.node.node_position_z.append(self.nodeattribute_list[i].nodez) # 点Z坐标信息
            nodeMsg.node.node_yaw.append(self.nodeattribute_list[i].nodeyaw)      # 点偏航角信息
            nodeMsg.node.node_attribute.append(0)                                 # 未开放属性，请赋值0，注意：不能为空！！！不能为空！！！不能为空！！！
            nodeMsg.node.undefined.append(0)
            nodeMsg.node.node_state_2.append(0)
            nodeMsg.node.node_state_3.append(0)
        self.pub_qt_node.publish(nodeMsg)

        self.index += 1
        edgeMsg.seq.data = "index:" + str(self.index) + ";"
        for i in range(len(self.edgeattribute_list)):
            edgeMsg.edge.edge_name.append(self.edgeattribute_list[i].edgename)        # 边名称
            edgeMsg.edge.start_node_name.append(self.edgeattribute_list[i].edgestart) # 起始节点名称
            edgeMsg.edge.end_node_name.append(self.edgeattribute_list[i].edgeend)     # 结束节点名称
            edgeMsg.edge.edge_length.append(0)
            edgeMsg.edge.dog_speed.append(0.5)              # 机器狗走这条边的速度（0-1）
            edgeMsg.edge.edge_state_2.append(0)             # 0：停止 1：避障 3：重规划  遇到障碍物时。建议赋值0

            edgeMsg.edge.dog_stats.append(0)                # 未开放属性，请赋值0，注意：不能为空！！！不能为空！！！不能为空！！！
            edgeMsg.edge.dog_back_stats.append(0)
            edgeMsg.edge.edge_state.append(0)
            edgeMsg.edge.edge_state_1.append(0)
            edgeMsg.edge.edge_state_3.append(0)
            edgeMsg.edge.edge_state_4.append(0)
        self.pub_qt_edge.publish(edgeMsg)

    # 删除所有节点函数
    def delete_all_node(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 1                                         # 1是删除指令
        msg.attribute = 1                                       # 1表示删除节点
        msg.node_edge_name.append(999)                          # 999表示删除所有节点
        self.pub_qt_command.publish(msg)

    # 删除所有边函数
    def delete_all_edge(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 1                                         # 1是删除指令
        msg.attribute = 2                                       # 2表示删除边
        msg.node_edge_name.append(999)                          # 999表示删除所有边
        self.pub_qt_command.publish(msg)

    # 暂停导航函数
    def pause_navigation(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 13                                        # 13是暂停导航命令
        self.pub_qt_command.publish(msg)

    # 恢复导航函数
    def recover_navigation(self):
        self.index += 1
        msg = QtCommand()
        msg.seq.data = f"index:{self.index};"
        msg.command = 14                                        # 14是恢复导航命令
        self.pub_qt_command.publish(msg)

    # 按键检测函数
    def key_detection(self):
        fd = sys.stdin.fileno()                                 # 获取标准输入文件描述符
        old_settings = termios.tcgetattr(fd)                    # 保存当前终端的标准输入设置
        try:
            tty.setcbreak(sys.stdin.fileno())                   # 设置终端为字符模式，确保信号输入可以识别
            ch = sys.stdin.read(1)                              # 读取一个字符
            self.get_logger().info(f"Key {ch} pressed.")        # 记录按键信息
            
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)  # 恢复终端设置
        return ch                                                   # 返回按键字符


    # 按键执行回调函数
    def key_execute_callback(self):
        current_key = self.key_detection()                              # 检测按键
        
        if current_key == 'q':
            self.close_all_node()                                       # 关闭所有节点
                                     
        elif current_key == 'w':
            self.delete_all_node()                                      # 删除所有节点
            self.delete_all_edge()                                      # 删除所有边
            self.nodeattribute_list.clear()                             # 清除节点属性列表
            self.edgeattribute_list.clear()                             # 清除边属性列表
            self.start_mapping()                                        # 开始建图
            self.nodename = 0                                           # 重置节点名称

        elif current_key == 'e':
            self.end_mapping()                  # 结束建图
        
        elif current_key == 'a':
            self.start_relocation()             # 开始重定位
            self.start_navigation()             # 开始导航
            self.init_pose()                    # 初始化姿态
            self.default_navigation()           # 默认导航

        elif current_key == 's':
            self.pause_navigation()             # 暂停导航

        elif current_key == 'd':
            self.recover_navigation()           # 恢复导航

        elif current_key == 'z':
            self.delete_all_node()              # 删除所有节点
            self.delete_all_edge()              # 删除所有边
            self.start_relocation()             # 开始重定位
            self.start_navigation()             # 开始导航
            self.init_pose()                    # 初始化姿态

        elif current_key == 'x':
            self.add_nodeand_edge()             # 添加节点和边

        elif current_key == 'c':
            self.save_nodeand_edge()            # 保存节点和边
            self.nodeattribute_list.clear()     # 清除节点属性列表
            self.edgeattribute_list.clear()     # 清除边属性列表
            self.nodename = 0                   # 重置节点名称

        elif current_key == 'v':
            self.delete_all_node()              # 删除所有节点
            self.delete_all_edge()              # 删除所有边
            self.nodename = 0                   # 重置节点名称



def main(args=None):
    rclpy.init(args=args)
    slam_demo =SlamDemo('unitree_slam_node')
    try:
        rclpy.spin(slam_demo)

    finally:
        if not rclpy.ok():
            print("ROS 2 context already shutdown.")
        else:
            rclpy.shutdown()



if __name__ == '__main__':
    main()

