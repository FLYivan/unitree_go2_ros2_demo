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



colcon build --packages-select go2_bringup

ros2 launch go2_bringup go2_m_explore.launch.py 	
ros2 launch go2_bringup test_explore.launch.py 

ros2 run go2_bringup go2_move


2、还是调大定时器间隔，——有没有定时器都差不多
3、在卡死时用遥控器模拟外部控制信号

解决问题
1、死胡同转向，头朝死胡同
2、死胡同逃离
3、背后负载
4、根据参数文件再看下

花了大量时间调参，以适应真实传感器和真实环境







# 调试时记录的问题


2、定时器的高频启动有助于避免navetopose的过程中，由于长路径的索死，但会把更多的点因为aborted加入黑名单
3、大理石 瓷砖墙面



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
5、机器人被困在局部最小值中（例如陷入死角）
     机器人想要到达目标G
    但被U形障碍物包围
    需要先远离目标才能绕过障碍到达目标
    但局部规划可能总是倾向于向目标方向移动
6、源代码在实际工况验证时，不合理，比如定时器
6、后退行为的触发
后退通常是作为恢复行为(Recovery Behavior)的一部分
不是常规路径规划的选项
只在遇到问题时作为应急手段
这就解释了为什么在遇到死角时：
局部规划器会陷入局部最小值
不会自动选择倒退作为解决方案
需要触发恢复行为才会执行后退动作
这确实是 Nav2 局部规划器的一个局限性。在某些场景下,允许规划器考虑后退路径可能会是更好的选择。

    为什么会陷入
    局部路径规划器往往只考虑局部环境
    基于梯度下降等方法可能会:
    不断尝试向目标靠近
    被障碍物阻挡
    来回摆动或停滞不前



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
1）调整动作：调小局部地图中的footprint 膨胀层——将膨胀层从0.1降低到0.05,有助于从狭窄空间逃脱
2）结论：
3）备注：

6、explore——progress_timeout
1）调整动作：将return改break
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

