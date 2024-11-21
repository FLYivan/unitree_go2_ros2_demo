#! /usr/bin/env python3  # 指定解释器
# Copyright 2019 Samsung Research America  # 版权声明
#
# Licensed under the Apache License, Version 2.0 (the "License");  # 根据Apache许可证2.0授权
# you may not use this file except in compliance with the License.  # 除非遵守许可证，否则不得使用此文件
# You may obtain a copy of the License at  # 您可以在以下网址获取许可证副本
#
#     http://www.apache.org/licenses/LICENSE-2.0  # http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software  # 除非适用法律要求或书面同意，否则软件
# distributed under the License is distributed on an "AS IS" BASIS,  # 在许可证下分发的软件是基于“原样”分发的
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  # 没有任何形式的明示或暗示的保证或条件
# See the License for the specific language governing permissions and  # 请参阅许可证以了解特定语言的权限和
# limitations under the License.  # 限制
#

import sys  # 导入sys模块
import time  # 导入time模块

from action_msgs.msg import GoalStatus  # 从action_msgs.msg模块导入GoalStatus类
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped  # 从geometry_msgs.msg模块导入PoseStamped, PoseWithCovarianceStamped类
# from nav2_msgs.action import FollowWaypoints  # 从nav2_msgs.action模块导入FollowWaypoints类
from nav2_msgs.action import NavigateToPose  # 从nav2_msgs.action模块导入NavigateToPose类
from nav2_msgs.srv import ManageLifecycleNodes  # 从nav2_msgs.srv模块导入ManageLifecycleNodes类
from nav2_msgs.srv import GetCostmap  # 从nav2_msgs.srv模块导入GetCostmap类
from nav2_msgs.msg import Costmap  # 从nav2_msgs.msg模块导入Costmap类
from nav_msgs.msg  import OccupancyGrid  # 从nav_msgs.msg模块导入OccupancyGrid类
from nav_msgs.msg import Odometry  # 从nav_msgs.msg模块导入Odometry类

import rclpy  # 导入rclpy模块
from rclpy.action import ActionClient  # 从rclpy.action模块导入ActionClient类
from rclpy.node import Node  # 从rclpy.node模块导入Node类
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy  # 从rclpy.qos模块导入QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy类
from rclpy.qos import QoSProfile  # 从rclpy.qos模块导入QoSProfile类

from enum import Enum  # 从enum模块导入Enum类

import numpy as np  # 导入numpy模块

import math  # 导入math模块

OCC_THRESHOLD = 10  # 设置OCC_THRESHOLD为10
MIN_FRONTIER_SIZE = 5  # 设置MIN_FRONTIER_SIZE为5

class Costmap2d():  # 定义Costmap2d类
    class CostValues(Enum):  # 定义CostValues枚举
        FreeSpace = 0  # FreeSpace值为0
        InscribedInflated = 253  # InscribedInflated值为253
        LethalObstacle = 254  # LethalObstacle值为254
        NoInformation = 255  # NoInformation值为255
    
    def __init__(self, map):  # 定义Costmap2d类的初始化方法
        self.map = map  # 设置map属性为传入的map

    def getCost(self, mx, my):  # 定义getCost方法
        return self.map.data[self.__getIndex(mx, my)]  # 返回map中(mx, my)位置的值

    def getSize(self):  # 定义getSize方法
        return (self.map.metadata.size_x, self.map.metadata.size_y)  # 返回map的大小

    def getSizeX(self):  # 定义getSizeX方法
        return self.map.metadata.size_x  # 返回map的宽度

    def getSizeY(self):  # 定义getSizeY方法
        return self.map.metadata.size_y  # 返回map的高度

    def __getIndex(self, mx, my):  # 定义__getIndex方法
        return my * self.map.metadata.size_x + mx  # 返回(mx, my)在map中的索引

class OccupancyGrid2d():  # 定义OccupancyGrid2d类
    class CostValues(Enum):  # 定义CostValues枚举
        FreeSpace = 0  # FreeSpace值为0
        InscribedInflated = 100  # InscribedInflated值为100
        LethalObstacle = 100  # LethalObstacle值为100
        NoInformation = -1  # NoInformation值为-1

    def __init__(self, map):  # 定义OccupancyGrid2d类的初始化方法
        self.map = map  # 设置map属性为传入的map

    def getCost(self, mx, my):  # 定义getCost方法
        return self.map.data[self.__getIndex(mx, my)]  # 返回map中(mx, my)位置的值

    def getSize(self):  # 定义getSize方法
        return (self.map.info.width, self.map.info.height)  # 返回map的大小

    def getSizeX(self):  # 定义getSizeX方法
        return self.map.info.width  # 返回map的宽度

    def getSizeY(self):  # 定义getSizeY方法
        return self.map.info.height  # 返回map的高度

    def mapToWorld(self, mx, my):  # 定义mapToWorld方法
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution  # 计算世界坐标系中的x值
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution  # 计算世界坐标系中的y值

        return (wx, wy)  # 返回世界坐标系中的坐标

    def worldToMap(self, wx, wy):  # 定义worldToMap方法
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):  # 如果世界坐标系中的坐标小于地图原点的坐标
            raise Exception("World coordinates out of bounds")  # 抛出异常，世界坐标系中的坐标超出范围

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)  # 计算地图坐标系中的x值
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)  # 计算地图坐标系中的y值
        
        if  (my > self.map.info.height or mx > self.map.info.width):  # 如果地图坐标系中的坐标大于地图的大小
            raise Exception("Out of bounds")  # 抛出异常，坐标超出范围

        return (mx, my)  # 返回地图坐标系中的坐标

    def __getIndex(self, mx, my):  # 定义__getIndex方法
        return my * self.map.info.width + mx  # 返回(mx, my)在map中的索引

class FrontierCache():  # 定义FrontierCache类
    cache = {}  # 定义cache属性为一个空字典

    def getPoint(self, x, y):  # 定义getPoint方法
        idx = self.__cantorHash(x, y)  # 计算(x, y)的cantor哈希值

        if idx in self.cache:  # 如果cantor哈希值在cache中
            return self.cache[idx]  # 返回cache中的值

        self.cache[idx] = FrontierPoint(x, y)  # 设置cache中的值为FrontierPoint(x, y)
        return self.cache[idx]  # 返回cache中的值

    def __cantorHash(self, x, y):  # 定义__cantorHash方法
        return (((x + y) * (x + y + 1)) / 2) + y  # 返回(x, y)的cantor哈希值

    def clear(self):  # 定义clear方法
        self.cache = {}  # 清空cache

class FrontierPoint():  # 定义FrontierPoint类
    def __init__(self, x, y):  # 定义FrontierPoint类的初始化方法
        self.classification = 0  # 设置classification属性为0
        self.mapX = x  # 设置mapX属性为x
        self.mapY = y  # 设置mapY属性为y

def centroid(arr):  # 定义centroid函数
    arr = np.array(arr)  # 将arr转换为numpy数组
    length = arr.shape[0]  # 获取arr的长度
    sum_x = np.sum(arr[:, 0])  # 计算arr中所有元素的x值的和
    sum_y = np.sum(arr[:, 1])  # 计算arr中所有元素的y值的和
    return sum_x/length, sum_y/length  # 返回x值的平均值和y值的平均值

def findFree(mx, my, costmap):  # 定义findFree函数
    fCache = FrontierCache()  # 创建FrontierCache实例

    bfs = [fCache.getPoint(mx, my)]  # 创建一个列表，包含一个FrontierPoint实例

    while len(bfs) > 0:  # 当bfs的长度大于0时
        loc = bfs.pop(0)  # 从bfs中取出第一个元素

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:  # 如果costmap中(loc.mapX, loc.mapY)位置的值等于FreeSpace值
            return (loc.mapX, loc.mapY)  # 返回(loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):  # 对loc的所有邻居进行遍历
            if n.classification & PointClassification.MapClosed.value == 0:  # 如果n的classification属性与MapClosed值的按位与结果为0
                n.classification = n.classification | PointClassification.MapClosed.value  # 设置n的classification属性为n的classification属性与MapClosed值的按位或结果
                bfs.append(n)  # 将n添加到bfs中

    return (mx, my)  # 如果没有找到FreeSpace值，返回(mx, my)

def getFrontier(pose, costmap, logger):  # 定义getFrontier函数
    fCache = FrontierCache()  # 创建FrontierCache实例

    fCache.clear()  # 清空fCache

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)  # 将pose的世界坐标系中的x值和y值转换为地图坐标系中的x值和y值

    freePoint = findFree(mx, my, costmap)  # 查找FreeSpace值
    start = fCache.getPoint(freePoint[0], freePoint[1])  # 创建一个FrontierPoint实例
    start.classification = PointClassification.MapOpen.value  # 设置start的classification属性为MapOpen值
    mapPointQueue = [start]  # 创建一个列表，包含start

    frontiers = []  # 创建一个空列表

    while len(mapPointQueue) > 0:  # 当mapPointQueue的长度大于0时
        p = mapPointQueue.pop(0)  # 从mapPointQueue中取出第一个元素

        if p.classification & PointClassification.MapClosed.value != 0:  # 如果p的classification属性与MapClosed值的按位与结果不为0
            continue  # 继续下一次循环

        if isFrontierPoint(p, costmap, fCache):  # 如果p是边界点
            p.classification = p.classification | PointClassification.FrontierOpen.value  # 设置p的classification属性为p的classification属性与FrontierOpen值的按位或结果
            frontierQueue = [p]  # 创建一个列表，包含p
            newFrontier = []  # 创建一个空列表

            while len(frontierQueue) > 0:  # 当frontierQueue的长度大于0时
                q = frontierQueue.pop(0)  # 从frontierQueue中取出第一个元素

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:  # 如果q的classification属性与(MapClosed值的按位或结果 | FrontierClosed值的按位或结果)的按位与结果不为0
                    continue  # 继续下一次循环

                if isFrontierPoint(q, costmap, fCache):  # 如果q是边界点
                    newFrontier.append(q)  # 将q添加到newFrontier中

                    for w in getNeighbors(q, costmap, fCache):  # 对q的所有邻居进行遍历
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:  # 如果w的classification属性与(FrontierOpen值的按位或结果 | FrontierClosed值的按位或结果 | MapClosed值的按位或结果)的按位与结果为0
                            w.classification = w.classification | PointClassification.FrontierOpen.value  # 设置w的classification属性为w的classification属性与FrontierOpen值的按位或结果
                            frontierQueue.append(w)  # 将w添加到frontierQueue中

                q.classification = q.classification | PointClassification.FrontierClosed.value  # 设置q的classification属性为q的classification属性与FrontierClosed值的按位或结果

            
            newFrontierCords = []  # 创建一个空列表
            for x in newFrontier:  # 对newFrontier中的所有元素进行遍历
                x.classification = x.classification | PointClassification.MapClosed.value  # 设置x的classification属性为x的classification属性与MapClosed值的按位或结果
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))  # 将x的地图坐标系中的x值和y值转换为世界坐标系中的x值和y值

            if len(newFrontier) > MIN_FRONTIER_SIZE:  # 如果newFrontier的长度大于MIN_FRONTIER_SIZE
                frontiers.append(centroid(newFrontierCords))  # 将newFrontierCords的中心点添加到frontiers中

        for v in getNeighbors(p, costmap, fCache):  # 对p的所有邻居进行遍历
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:  # 如果v的classification属性与(MapOpen值的按位或结果 | MapClosed值的按位或结果)的按位与结果为0
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):  # 如果v的所有邻居中有一个的costmap中的值等于FreeSpace值
                    v.classification = v.classification | PointClassification.MapOpen.value  # 设置v的classification属性为v的classification属性与MapOpen值的按位或结果
                    mapPointQueue.append(v)  # 将v添加到mapPointQueue中

        p.classification = p.classification | PointClassification.MapClosed.value  # 设置p的classification属性为p的classification属性与MapClosed值的按位或结果

    return frontiers  # 返回frontiers
        
def getNeighbors(point, costmap, fCache):  # 定义getNeighbors函数
    neighbors = []  # 创建一个空列表

    for x in range(point.mapX - 1, point.mapX + 2):  # 对point的所有邻居进行遍历
        for y in range(point.mapY - 1, point.mapY + 2):  # 对point的所有邻居进行遍历
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):  # 如果x大于0且x小于costmap的宽度且y大于0且y小于costmap的高度
                neighbors.append(fCache.getPoint(x, y))  # 将邻居点添加到neighbors列表中

    return neighbors  # 返回neighbors列表

def isFrontierPoint(point, costmap, fCache):  # 定义isFrontierPoint函数
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:  # 如果point的costmap值不等于NoInformation值
        return False  # 返回False

    hasFree = False  # 初始化hasFree为False
    for n in getNeighbors(point, costmap, fCache):  # 对point的所有邻居进行遍历
        cost = costmap.getCost(n.mapX, n.mapY)  # 获取邻居点的costmap值

        if cost > OCC_THRESHOLD:  # 如果cost大于OCC_THRESHOLD
            return False  # 返回False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:  # 如果cost等于FreeSpace值
            hasFree = True  # 设置hasFree为True

    return hasFree  # 返回hasFree

class PointClassification(Enum):  # 定义PointClassification枚举类
    MapOpen = 1  # MapOpen值为1
    MapClosed = 2  # MapClosed值为2
    FrontierOpen = 4  # FrontierOpen值为4
    FrontierClosed = 8  # FrontierClosed值为8

class WaypointFollowerTest(Node):  # 定义WaypointFollowerTest类，继承自Node

    def __init__(self):  # 定义初始化函数
        super().__init__(node_name='nav2_waypoint_tester', namespace='')  # 调用父类的初始化函数
        self.waypoints = None  # 初始化waypoints为None
        self.readyToMove = True  # 初始化readyToMove为True
        self.currentPose = None  # 初始化currentPose为None
        self.lastWaypoint = None  # 初始化lastWaypoint为None

        # 定义地图坐标id参数
        self.declare_parameter('map_frame_id', 'map_slamtoolbox')
        self.map_frame_id_cmd = self.get_parameter('map_frame_id').get_parameter_value().string_value
        # 定义odom话题参数
        self.declare_parameter('odom_topic', 'lio_sam_ros2/dogOdomProcess/DogOdomGlobal')
        self.odom_topic_cmd = self.get_parameter('odom_topic').get_parameter_value().string_value
        # 定义map话题参数
        self.declare_parameter('map_topic', 'map_slamtoolbox_go2')
        self.map_topic_cmd = self.get_parameter('map_topic').get_parameter_value().string_value

   

        # self.action_client = ActionClient(self, FollowWaypoints, 'FollowWaypoints')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')  # 创建NavigateToPose的ActionClient
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)  # 创建initialpose的发布者

        self.costmapClient = self.create_client(GetCostmap, '/global_costmap/get_costmap')  # 创建GetCostmap的客户端
        while not self.costmapClient.wait_for_service(timeout_sec=1.0):  # 等待服务可用
            self.info_msg('service not available, waiting again...')  # 打印信息消息
        self.initial_pose_received = False  # 初始化initial_pose_received为False
        self.goal_handle = None  # 初始化goal_handle为None

        pose_qos = QoSProfile(  # 创建QoSProfile
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # 设置durability为TRANSIENT_LOCAL
          reliability=QoSReliabilityPolicy.RELIABLE,  # 设置reliability为RELIABLE
          # reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,  # 设置history为KEEP_LAST
          depth=1)  # 设置depth为1

        self.model_pose_sub = self.create_subscription(Odometry, self.odom_topic_cmd, self.poseCallback, 10)  # 创建/odom的订阅者   modify

        # self.costmapSub = self.create_subscription(Costmap(), '/global_costmap/costmap_raw', self.costmapCallback, pose_qos)  
        self.costmapSub = self.create_subscription(OccupancyGrid(), self.map_topic_cmd, self.occupancyGridCallback, pose_qos)  # 创建/map的订阅者  modify
        self.costmap = None  # 初始化costmap为None

        self.get_logger().info('Running Waypoint Test')  # 打印信息消息

    def occupancyGridCallback(self, msg):  # 定义occupancyGridCallback函数
        self.costmap = OccupancyGrid2d(msg)  # 将msg转换为OccupancyGrid2d并赋值给costmap

    def moveToFrontiers(self):  # 定义moveToFrontiers函数
        frontiers = getFrontier(self.currentPose, self.costmap, self.get_logger())  # 获取边界点

        if len(frontiers) == 0:  # 如果没有边界点
            self.info_msg('No More Frontiers')  # 打印信息消息
            return  # 返回

        location = None  # 初始化location为None
        largestDist = 0  # 初始化largestDist为0
        for f in frontiers:  # 对所有边界点进行遍历
            dist = math.sqrt(((f[0] - self.currentPose.position.x)**2) + ((f[1] - self.currentPose.position.y)**2))  # 计算边界点与当前位置的距离
            if  dist > largestDist:  # 如果距离大于largestDist
                largestDist = dist  # 更新largestDist
                location = [f]  # 更新location

        #worldFrontiers = [self.costmap.mapToWorld(f[0], f[1]) for f in frontiers]
        self.info_msg(f'World points {location}')  # 打印信息消息
        self.setWaypoints(location)  # 设置路径点

        # action_request = FollowWaypoints.Goal()
        action_request = NavigateToPose.Goal()  # 创建NavigateToPose的Goal
        # action_request.poses = self.waypoints
        action_request.pose = self.waypoints[0]  # 设置Goal的pose

        self.info_msg('Sending goal request...')  # 打印信息消息
        send_goal_future = self.action_client.send_goal_async(action_request)  # 发送Goal请求
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)  # 等待请求完成
            self.goal_handle = send_goal_future.result()  # 获取结果
        except Exception as e:  # 捕获异常
            self.error_msg('Service call failed %r' % (e,))  # 打印错误消息

        if not self.goal_handle.accepted:  # 如果Goal未被接受
            self.error_msg('Goal rejected')  # 打印错误消息
            return  # 返回

        self.info_msg('Goal accepted')  # 打印信息消息

        get_result_future = self.goal_handle.get_result_async()  # 获取结果的异步请求

        # self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        self.info_msg("Waiting for 'NavigateToPose' action to complete")  # 打印信息消息
        try:
            rclpy.spin_until_future_complete(self, get_result_future)  # 等待请求完成
            status = get_result_future.result().status  # 获取状态
            result = get_result_future.result().result  # 获取结果
        except Exception as e:  # 捕获异常
            self.error_msg('Service call failed %r' % (e,))  # 打印错误消息

        #self.currentPose = self.waypoints[len(self.waypoints) - 1].pose

        self.moveToFrontiers()  # 移动到边界点

    def costmapCallback(self, msg):  # 定义costmapCallback函数
        self.costmap = Costmap2d(msg)  # 将msg转换为Costmap2d并赋值给costmap

        unknowns = 0  # 初始化unknowns为0
        for x in range(0, self.costmap.getSizeX()):  # 对costmap的所有x坐标进行遍历
            for y in range(0, self.costmap.getSizeY()):  # 对costmap的所有y坐标进行遍历
                if self.costmap.getCost(x, y) == 255:  # 如果costmap的值等于255
                    unknowns = unknowns + 1  # 增加unknowns的值
        self.get_logger().info(f'Unknowns {unknowns}')  # 打印信息消息
        self.get_logger().info(f'Got Costmap {len(getFrontier(None, self.costmap, self.get_logger()))}')  # 打印信息消息

    def dumpCostmap(self):  # 定义dumpCostmap函数
        costmapReq = GetCostmap.Request()  # 创建GetCostmap的请求
        self.get_logger().info('Requesting Costmap')  # 打印信息消息
        costmap = self.costmapClient.call(costmapReq)  # 发送请求并获取结果
        self.get_logger().info(f'costmap resolution {costmap.specs.resolution}')  # 打印信息消息

    def setInitialPose(self, pose):  # 定义setInitialPose函数
        self.init_pose = PoseWithCovarianceStamped()  # 创建PoseWithCovarianceStamped对象
        self.init_pose.pose.pose.position.x = pose[0]  # 设置x坐标
        self.init_pose.pose.pose.position.y = pose[1]  # 设置y坐标
        self.init_pose.header.frame_id = self.map_frame_id_cmd  # 设置frame_id                      modify
        self.currentPose = self.init_pose.pose.pose  # 设置currentPose
        self.publishInitialPose()  # 发布初始位姿
        time.sleep(5)  # 等待5秒

    def poseCallback(self, msg):  # 定义poseCallback函数
        if (not self.initial_pose_received):  # 如果未接收到初始位姿
          self.info_msg('Received amcl_pose')  # 打印信息消息
        self.currentPose = msg.pose.pose  # 设置currentPose
        self.initial_pose_received = True  # 设置initial_pose_received为True


    def setWaypoints(self, waypoints):  # 定义setWaypoints函数
        self.waypoints = []  # 初始化waypoints为空列表
        for wp in waypoints:  # 对所有路径点进行遍历
            msg = PoseStamped()  # 创建PoseStamped对象
            msg.header.frame_id = self.map_frame_id_cmd  # 设置frame_id                     modify
            msg.pose.position.x = wp[0]  # 设置x坐标
            msg.pose.position.y = wp[1]  # 设置y坐标
            msg.pose.orientation.w = 1.0  # 设置方向
            self.waypoints.append(msg)  # 将路径点添加到waypoints中

    def run(self, block):  # 定义run函数
        if not self.waypoints:  # 如果没有设置路径点
            rclpy.error_msg('Did not set valid waypoints before running test!')  # 打印错误消息
            return False  # 返回False

        while not self.action_client.wait_for_server(timeout_sec=1.0):  # 等待服务器可用
            # self.info_msg("'FollowWaypoints' action server not available, waiting...")
            self.info_msg("'NavigateToPose' action server not available, waiting...")  # 打印信息消息

        # action_request = FollowWaypoints.Goal()
        action_request = NavigateToPose.Goal()  # 创建NavigateToPose的Goal
        # action_request.poses = self.waypoints
        action_request.pose = self.waypoints[0]  # 设置Goal的pose

        self.info_msg('Sending goal request...')  # 打印信息消息
        send_goal_future = self.action_client.send_goal_async(action_request)  # 发送Goal请求
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)  # 等待请求完成
            self.goal_handle = send_goal_future.result()  # 获取结果
        except Exception as e:  # 捕获异常
            self.error_msg('Service call failed %r' % (e,))  # 打印错误消息

        if not self.goal_handle.accepted:  # 如果Goal未被接受
            self.error_msg('Goal rejected')  # 打印错误消息
            return False  # 返回False

        self.info_msg('Goal accepted')  # 打印信息消息
        if not block:  # 如果不阻塞
            return True  # 返回True

        get_result_future = self.goal_handle.get_result_async()  # 获取结果的异步请求

        # self.info_msg("Waiting for 'FollowWaypoints' action to complete")
        self.info_msg("Waiting for 'NavigateToPose' action to complete")  # 打印信息消息
        try:
            rclpy.spin_until_future_complete(self, get_result_future)  # 等待请求完成
            status = get_result_future.result().status  # 获取状态
            result = get_result_future.result().result  # 获取结果
        except Exception as e:  # 捕获异常
            self.error_msg('Service call failed %r' % (e,))  # 打印错误消息

        if status != GoalStatus.STATUS_SUCCEEDED:  # 如果状态不等于STATUS_SUCCEEDED
            self.info_msg('Goal failed with status code: {0}'.format(status))  # 打印信息消息
            return False  # 返回False
        if len(result.missed_waypoints) > 0:  # 如果有未处理的路径点
            self.info_msg('Goal failed to process all waypoints, missed {0} wps.'.format(len(result.missed_waypoints)))  # 打印信息消息
            return False  # 返回False

        self.info_msg('Goal succeeded!')  # 打印信息消息
        return True  # 返回True

    def publishInitialPose(self):  # 定义publishInitialPose函数
        self.initial_pose_pub.publish(self.init_pose)  # 发布初始位姿

    def shutdown(self):  # 定义shutdown函数
        self.info_msg('Shutting down')  # 打印信息消息

        self.action_client.destroy()  # 销毁action_client
        # self.info_msg('Destroyed FollowWaypoints action client')
        self.info_msg('Destroyed NavigateToPose action client')  # 打印信息消息

        transition_service = 'lifecycle_manager_navigation/manage_nodes'  # 设置transition_service      
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)  # 创建ManageLifecycleNodes的客户端
        while not mgr_client.wait_for_service(timeout_sec=1.0):  # 等待服务可用
            self.info_msg(transition_service + ' service not available, waiting...')  # 打印信息消息

        req = ManageLifecycleNodes.Request()  # 创建ManageLifecycleNodes的请求
        req.command = ManageLifecycleNodes.Request().SHUTDOWN  # 设置请求命令为SHUTDOWN
        future = mgr_client.call_async(req)  # 发送异步请求
        try:
            rclpy.spin_until_future_complete(self, future)  # 等待请求完成
            future.result()  # 获取结果
        except Exception as e:  # 捕获异常
            self.error_msg('%s service call failed %r' % (transition_service, e,))  # 打印错误消息

        self.info_msg('{} finished'.format(transition_service))  # 打印信息消息

        transition_service = 'lifecycle_manager_localization/manage_nodes'  # 设置transition_service
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)  # 创建ManageLifecycleNodes的客户端
        while not mgr_client.wait_for_service(timeout_sec=1.0):  # 等待服务可用
            self.info_msg(transition_service + ' service not available, waiting...')  # 打印信息消息

        req = ManageLifecycleNodes.Request()  # 创建ManageLifecycleNodes的请求
        req.command = ManageLifecycleNodes.Request().SHUTDOWN  # 设置请求命令为SHUTDOWN
        future = mgr_client.call_async(req)  # 发送异步请求
        try:
            rclpy.spin_until_future_complete(self, future)  # 等待请求完成
            future.result()  # 获取结果
        except Exception as e:  # 捕获异常
            self.error_msg('%s service call failed %r' % (transition_service, e,))  # 打印错误消息

        self.info_msg('{} finished'.format(transition_service))  # 打印信息消息

    def cancel_goal(self):  # 定义cancel_goal函数
        cancel_future = self.goal_handle.cancel_goal_async()  # 取消Goal的异步请求
        rclpy.spin_until_future_complete(self, cancel_future)  # 等待请求完成

    def info_msg(self, msg: str):  # 定义info_msg函数
        self.get_logger().info(msg)  # 打印信息消息

    def warn_msg(self, msg: str):  # 定义warn_msg函数
        self.get_logger().warn(msg)  # 打印警告消息

    def error_msg(self, msg: str):  # 定义error_msg函数
        self.get_logger().error(msg)  # 打印错误消息


def main(argv=sys.argv[1:]):  # 定义main函数
    rclpy.init()  # 初始化rclpy

    # wait a few seconds to make sure entire stacks are up
    #time.sleep(10)

    # wps = [[-0.52, -0.54], [0.58, -0.55], [0.58, 0.52]]
    # starting_pose = [-2.0, -0.5]  # 设置初始位姿                        modify

    test = WaypointFollowerTest()  # 创建WaypointFollowerTest对象
    #test.dumpCostmap()
    # test.setWaypoints(wps)

    retry_count = 0  # 初始化retry_count为0
    retries = 2000  # 设置retries为2000
    while not test.initial_pose_received and retry_count <= retries:  # 如果未接收到初始位姿且重试次数小于等于retries
        retry_count += 1  # 增加retry_count
        test.info_msg('Setting initial pose')  # 打印信息消息
        # test.setInitialPose(starting_pose)  # 设置初始位姿                          modify
        test.info_msg('Waiting for amcl_pose to be received')  # 打印信息消息
        rclpy.spin_once(test, timeout_sec=1.0)  # 等待poseCallback

    while test.costmap == None:  # 如果costmap为None
        test.info_msg('Getting initial map')  # 打印信息消息
        rclpy.spin_once(test, timeout_sec=1.0)  # 等待occupancyGridCallback

    test.moveToFrontiers()  # 移动到边界点

    rclpy.spin(test)  # 运行rclpy
    # result = test.run(True)
    # assert result

    # # preempt with new point
    # test.setWaypoints([starting_pose])
    # result = test.run(False)
    # time.sleep(2)
    # test.setWaypoints([wps[1]])
    # result = test.run(False)

    # # cancel
    # time.sleep(2)
    # test.cancel_goal()

    # # a failure case
    # time.sleep(2)
    # test.setWaypoints([[100.0, 100.0]])
    # result = test.run(True)
    # assert not result
    # result = not result

    # test.shutdown()
    # test.info_msg('Done Shutting Down.')

    # if not result:
    #     test.info_msg('Exiting failed')
    #     exit(1)
    # else:
    #     test.info_msg('Exiting passed')
    #     exit(0)


if __name__ == '__main__':  # 如果是主程序
    main()  # 调用main函数
