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


# 高度数据获取
colcon build --packages-select go2_bringup
ros2 run go2_bringup find_stairs

ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
ros2 launch explore_lite explore.launch.py

# 调试步骤
1、去除体素层 不能去除 pass
2、调小minimum_turning_radius pass
3、调整探索权重，优先距离,调小成本大小的权重 （目前参数可以有效行进，不要动了） not pass
4、重启时常   pass
4、PLAN超时，提高时间 pass
8、看下路由器延迟情况 20ms内极好 pass
9、分析smac算法每个参数的意义——pass
10、odom_topic要调整——pass
5、move模块报错——在move代码中增加初始化保护机制 pass

5、改变目标点容差，要再大点？


4、修改nav2_costmap_2d/costmap_2d_ros源码的transform_tolerance对应值,编译
——在源码中增加transform_tolerance打印，再进行观察
——调整failure_tolerance参数
——修改nav2_costmap_2d/costmap_2d_ros源码的transform_tolerance对应值，  没用，再调整

6、降低全局规划器频率（针对狭窄走廊）

[ERROR] [transformPoseInTargetFrame]: Extrapolation Error looking up target frame: Lookup would require extrapolation into the past.  Requested time 1733820797.110452 but the earliest data is at time 1733820823.080866, when looking up transform from frame [base] to frame [map]

[ERROR] [tf_help]: Transform data too old when converting from map to odom_slamtoolbox
[ERROR] [tf_help]: Data time: 1733823508s 564122457ns, Transform time: 1733823508s 210295278ns, tolerance is 0.300000
[ERROR] [tf_help]: Transform data too old when converting from map to odom_slamtoolbox
[ERROR] [tf_help]: Data time: 1733823508s 564122457ns, Transform time: 1733823508s 210295278ns, tolerance is 0.300000
[ERROR] [tf_help]: Transform data too old when converting from map to odom_slamtoolbox
[ERROR] [tf_help]: Data time: 1733823508s 564122457ns, Transform time: 1733823508s 210295278ns, tolerance is 0.300000
[ERROR] [tf_help]: Transform data too old when converting from map to odom_slamtoolbox
[ERROR] [tf_help]: Data time: 1733823508s 564122457ns, Transform time: 1733823508s 210295278ns, tolerance is 0.300000

[ERROR] [DWBLocalPlanner]: No valid trajectories out of 419! 
[ERROR] [DWBLocalPlanner]: 1.00: BaseObstacle/Trajectory Hits Obstacle.  在楼梯死胡同

9、往复跑的问题 还是要解决 调explore参数

10、走廊中间走，一些惩罚参苏


# 问题记录

1、向上楼梯，激光会认为障碍物
12、避障在长时间原地踏步？时才启动检测

a星计算超时
解决方案：
调整规划参数，如增加超时时间或最大迭代次数
优化启发式函数
降低地图分辨率
使用分层规划策略
在关键位置添加路径点，将长路径分解为多个短路径
适当放宽规划约束条件


狭窄走廊的路径规划更复杂
解决方案：
使用分层规划策略，先规划粗略路径，再细化局部路径——调整不同规划器频率，全局路径不需要频繁重新规划（1Hz）
针对狭窄区域使用特殊的启发式函数
在关键位置设置中间路径点
适当调整代价地图的膨胀参数



关键建议：
参数调整要循序渐进，每次只改动少量参数
需要在实际场景中反复测试和验证
记录不同参数组合的效果，找到最适合的配置
可以为不同类型的场景（开阔区域/狭窄走廊）准备不同的参数配置




# 不同场景参数配置方法
# A.狭窄通道


planner_server:
  ros__parameters:
    SmacHybrid:  
      # 额外的代价调整
      analytic_expansion_ratio: 3.5      # 分析扩展比例
      analytic_expansion_max_length: 3.0 # 最大扩展长度


      # 调整搜索参数
      allow_unknown: true                # 允许探索未知区域
      max_iterations: 500000             # 增加最大迭代次数                    2、如果路径仍不理想，进一步调整：
      max_on_approach_iterations: 1000   # 增加接近目标时的迭代次数
      smooth_path: true                  # 启用路径平滑                     3、最后优化平滑度：


      # 方向变化的代价权重
      non_straight_penalty: 1.2          # 非直线运动惩罚                   2、如果路径仍不理想，进一步调整：

      # reverse_penalty 和 change_penalty: 可以适当调小以允许更灵活的运动
      change_penalty: 0.05               # 方向改变惩罚                     2、如果路径仍不理想，进一步调整：
      reverse_penalty: 2.0               # 倒车惩罚
      
      # 通道相关参数
      minimum_turning_radius: 0.40       # 最小转弯半径，调小可以让机器人更容易通过狭窄区域                     3、最后优化平滑度：
      downsampling_factor: 1             # 降采样因子
      angle_quantization_bins: 64        # 角度量化bins数                   3、最后优化平滑度：


global_costmap:
  ros__parameters:
    inflation_layer:
      inflation_radius: 0.5              # 减小膨胀半径                          1、首先调整基础参数：
      cost_scaling_factor: 15.0          # 增大代价衰减因子， 使代价更快衰减        1、首先调整基础参数：
      






