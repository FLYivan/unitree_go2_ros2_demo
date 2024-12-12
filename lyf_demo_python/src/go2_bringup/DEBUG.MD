# 新建功能包
ros2 pkg create --license Apache-2.0 --build-type ament_python go2_ros2_basic

# 查看tf树
ros2 run rqt_tf_tree rqt_tf_tree --force-discover

# 构建节点
colcon build --packages-select go2_ros2_slam
colcon build --packages-select go2_description
colcon build --packages-select go2_bringup
colcon build --packages-select explore_lite

# go2位置校准
ros2 run go2_ros2_basic sport_demo 

# 建图
ros2 launch go2_bringup go2_slam_bringup.launch.py 
ros2 launch go2_bringup cloud_to_scan.launch.py
ros2 launch go2_bringup only_slam.launch.py 

ros2 run go2_bringup motion_to_tf 

# 地图保存
ros2 run nav2_map_server map_saver_cli -f room --ros-args --remap map:=/map_slamtoolbox_go2

# 启动nav2
ros2 launch go2_bringup cloud_to_scan.launch.py
ros2 launch go2_bringup go2_nav2_bringup.launch.py

# 启动slam联动nav2
ros2 launch hesai_ros_driver start.py

ros2 launch go2_bringup go2_m_explore.launch.py 	
ros2 launch go2_bringup test_explore.launch.py 

ros2 run go2_bringup go2_move

# 雷达驱动
ros2 launch hesai_ros_driver start.py


# 优化explore算法
colcon build --packages-select explore_lite

# nav2调试

colcon build --packages-select nav2_bringup
colcon build --packages-select nav2_navfn_planner
colcon build --packages-select nav2_smac_planner
colcon build --packages-select nav_2d_utils

colcon build --packages-select nav2_costmap_2d
colcon build --packages-select nav2_dwb_controller
colcon build --packages-select dwb_core


# 高度数据获取
colcon build --packages-select go2_bringup
ros2 run go2_bringup find_stairs

ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
ros2 launch explore_lite explore.launch.py

# 调试步骤
1、去除体素层 不能去除 pass
4、重启时常   pass
4、PLAN超时，提高时间 pass
8、看下路由器延迟情况 20ms内极好 pass
9、分析smac算法每个参数的意义——pass
10、odom_topic要调整——pass
5、move模块报错——在move代码中增加初始化保护机制 pass
6、[ERROR] [DWBLocalPlanner]: No valid trajectories out of 419! 
[ERROR] [DWBLocalPlanner]: 1.00: BaseObstacle/Trajectory Hits Obstacle.  在楼梯死胡同 ——pass
5、改变目标点容差，要再大点？——大一点才不会abord pass 




1、头朝外走3次，能走出来

1、调整动态tf发布频率，由20到100
2、调整tf的缓存
3、测试定时器重启最小值
4、测试使用纯追踪控制算法


1、调试如何才能在死胡同调头走出来
    5）调大激光最大值，能看的远一些
    1）调小minimum_turning_radius，最小转弯半径
    2）调整激光最小值
    3）调整footprint 膨胀层
    4）缩小整体膨胀层
    

4、测试合适的权重参数，不会往复跑
5、调整惩罚参数，让狗靠走廊中间走
    1)cost_penalty: 越大，离障碍物越远
6、降低全局规划器频率（针对狭窄走廊），看效果



# 代码逻辑分析
1、为什么定时器一触发就abord
2、为什么tf会突变0.3



测试1-1







# 问题记录

1、向上楼梯，激光会认为障碍物
2、
1）避障接管，当cmd_vel连续100下左右原地踏步
2）取消接管，当cmd_vel连续X下有值







关键建议：
参数调整要循序渐进，每次只改动少量参数
需要在实际场景中反复测试和验证
记录不同参数组合的效果，找到最适合的配置
可以为不同类型的场景（开阔区域/狭窄走廊）准备不同的参数配置



