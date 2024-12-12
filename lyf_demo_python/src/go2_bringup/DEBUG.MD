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

1)调高explore的 progress_timeout到50
2）其他参数参考12月10日早上数据

1、调整动态tf发布频率，由20到100 ——效果非常号
2、调整tf的缓存 30 测试无效
-planner
——ERROR] [transformPoseInTargetFrame]: Extrapolation Error looking up target frame: Lookup would require extrapolation into the past.  Requested time 1733970614.097182 but the earliest data is at time 1733970657.233162, when looking up transform from frame [base] to frame [map]
——[ERROR] [planner_server]: [compute_path_to_pose] [ActionServer] Action callback is still running and missed deadline to stop
bt-naviga
——[ERROR] [transformPoseInTargetFrame]: Extrapolation Error looking up target frame: Lookup would require extrapolation into the past.  Requested time 1733970614.097182 but the earliest data is at time 1733970634.512001, when looking up transform from frame [base] to frame [map]
explore
——[ERROR] [explore_node]: Extrapolation Error looking up robot pose: Lookup would require extrapolation into the future.  Requested time 1733970647.707089 but the latest data is at time 1733970647.579348, when looking up transform from frame [base] to frame [map]
4、测试使用纯追踪控制算法 ——效果不好
——尝试调大tf tolerance到1
5、调大explore的加入黑名单的时间阈值——测试 50正好
调大 100
调小 15
5、调整惩罚参数，让狗靠走廊中间走
    1)cost_penalty: 越大，离障碍物越远——效果不佳，放弃， 保持2.0
6、降低全局规划器频率（针对狭窄走廊），看效果
——planner的频率从20降到10hz  5hz不行


1、调试如何才能在死胡同调头走出来
    5）调大激光最大值，能看的远一些——可行，调到4和3.5
    1）调小minimum_turning_radius，最小转弯半径——无效必须保证0.05
    2）调整激光最小值
    3）调整footprint 膨胀层
    4）缩小整体膨胀层
    

colcon build --packages-select go2_bringup

ros2 launch go2_bringup go2_m_explore.launch.py 	
ros2 launch go2_bringup test_explore.launch.py 

ros2 run go2_bringup go2_move

4、测试合适的权重参数，不会往复跑
3、测试定时器重启最小值


# 代码逻辑分析
1、为什么定时器一触发就abord
定时器触发，会改变原来正在行进的路线
看日志，触发定时器后到底作了什么？
但同时，定时器触发会在锁死的情况下，重新规划路线并发送控制信号

为什么自动切换路径就卡死，定时器启动就有了？

3、explore超时了，会触发什么？

2、为什么tf会突变0.3
——第一处，直接调用了3d

3、关注网络问题

4、kazam坏了可能跟n卡驱动有关

5、记录调试手札


# 问题记录

1、向上楼梯，激光会认为障碍物
2、
1）避障接管，当cmd_vel连续100下左右原地踏步
2）取消接管，当cmd_vel连续X下有值

3、
1）定时器正常情况下不要启动
2）卡死情况下启动定时器
3）启动后仍不能解救，启动辟障程序
4）比章脱离后，要再次触发定时器启动规划


# 解决问题
1、复杂环境，算不出来
2、实例只有仿真，真实环境也非常简单
3、反复起停
4、大理石，玻璃墙面，对激光不友好，自探索时容易误检

关键建议：
参数调整要循序渐进，每次只改动少量参数
需要在实际场景中反复测试和验证
记录不同参数组合的效果，找到最适合的配置
可以为不同类型的场景（开阔区域/狭窄走廊）准备不同的参数配置



