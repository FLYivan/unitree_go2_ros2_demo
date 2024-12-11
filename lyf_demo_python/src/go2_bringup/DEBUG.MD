# 新建功能包
ros2 pkg create --license Apache-2.0 --build-type ament_python go2_ros2_basic

# 查看tf树
ros2 run rqt_tf_tree rqt_tf_tree --force-discover

# 构建节点
colcon build --packages-select go2_ros2_slam
colcon build --packages-select go2_description
colcon build --packages-select go2_bringup

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

6、 [ERROR] [transformPoseInTargetFrame]: Extrapolation Error looking up target frame: Lookup would require extrapolation into the past.  Requested time 1733820797.110452 but the earliest data is at time 1733820823.080866, when looking up transform from frame [base] to frame [map]——罕见错误，1、增大tf缓存，2、增大tf频率
调整动态tf发布频率，由20到100
2、调小minimum_turning_radius ——可以调到0.01吗？
6、降低全局规划器频率（针对狭窄走廊）



6、定时器重启有最小值？？
7、自探索时，Goal was aborted什么原因


4、修改nav2_costmap_2d/costmap_2d_ros源码的transform_tolerance对应值,编译
——在源码中增加transform_tolerance打印，再进行观察
——调整failure_tolerance参数
——修改nav2_costmap_2d/costmap_2d_ros源码的transform_tolerance对应值，  没用，再调整






9、往复跑的问题 还是要解决 调explore参数

10、走廊中间走，一些惩罚参苏



# 问题记录

1、向上楼梯，激光会认为障碍物








关键建议：
参数调整要循序渐进，每次只改动少量参数
需要在实际场景中反复测试和验证
记录不同参数组合的效果，找到最适合的配置
可以为不同类型的场景（开阔区域/狭窄走廊）准备不同的参数配置




# 不同场景参数配置方法
# A.狭窄通道

