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
colcon build --packages-select nav2_controller

# 高度数据获取
colcon build --packages-select go2_bringup
ros2 run go2_bringup find_stairs

ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
ros2 launch explore_lite explore.launch.py

# 调试步骤

1、调试如何才能在死胡同调头走出来
    3）调小局部地图中的footprint 膨胀层
    

colcon build --packages-select go2_bringup

ros2 launch go2_bringup go2_m_explore.launch.py 	
ros2 launch go2_bringup test_explore.launch.py 

ros2 run go2_bringup go2_move

2、确认tolerance问题，如果确认后，把日志打印注释

3、测试定时器重启最小值

test

# 代码逻辑分析
1、为什么定时器一触发就abord
定时器触发，会改变原来正在行进的路线
看日志，触发定时器后到底作了什么？
但同时，定时器触发会在锁死的情况下，重新规划路线并发送控制信号

为什么自动切换路径就卡死，定时器启动就有了？

3、explore超时了，会触发什么？

2、为什么tf会突变0.3
——第一处，直接调用了3d





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




# 参数调试结论
1、explore——progress_timeout
1）调整动作：由30调整到50；
2）结论：可行
3）备注：影响目标点纳入黑名单的等待时长，不能调大（100）或调小（15），否则影响正常探索

2、动态tf变换节点——tf发布频率
1）调整动作：由20hz到100hz
2）结论：效果非常好
3）备注：

3、动态tf变换节点——动态tf的缓存
1）调整动作：由10提升到30
2）结论：负面影响
3）备注：不要动

4、control节点——followpath插件
1）调整动作：改为纯追踪控制算法插件
2）结论：运动时左右摇摆，效果不好
3）备注：不要更换这个插件

5、planner——smacHybrid——cost_penalty
1）调整动作：调大
2）结论：效果不佳，
3）备注：无法起到原理障碍物规划路径的效果，且影响路径生成，需保持2.0

6、planner——expected_planner_frequency
1）调整动作：从20hz降到10hz,降到5hz
2）结论：不行，影响路径正常规划，保持20hz
3）备注：

6、局部和全局代价地图——激光最大值
1）调整动作：分别调高到4和3.5
2）结论：有效
3）备注：调整后能看远一点，有助于走出死胡同

6、planner——minimum_turning_radius
1）调整动作：从0.05下调到0.03,0.01
2）结论：都造成原地打转
3）备注：最小转完半径可能就是0.05

6、局部和全局代价地图——体素层
1）调整动作：用障碍物层代替体素层
2）结论：可以满足2d的规划，且解决z轴总是超出的报错
3）备注：需要同时加上静态层

6、explore——progress_timeout
1）调整动作：
2）结论：
3）备注：

6、explore——progress_timeout
1）调整动作：
2）结论：
3）备注：

6、explore——progress_timeout
1）调整动作：
2）结论：
3）备注：
