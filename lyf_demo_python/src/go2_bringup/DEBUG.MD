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
ros2 run nav2_map_server map_saver_cli -f f12-room --ros-args 
ros2 run nav2_map_server map_saver_cli -f b1-room

# 启动nav2
ros2 launch go2_bringup cloud_to_scan.launch.py
ros2 launch go2_bringup go2_nav2_bringup.launch.py

# 启动explore节点并同步建图
    # 拓展乌
    ros2 launch hesai_ros_driver start.py

    # 上位机
    ros2 launch go2_bringup go2_m_explore.launch.py 	


# 雷达驱动
ros2 launch hesai_ros_driver start.py


# 优化explore算法
colcon build --packages-select explore_lite

# 运动轨迹显示
colcon build --packages-select go2_bringup
ros2 run go2_bringup trajectory_visualizer

# 摄像头拉流驱动
ros2 run go2_bringup video_stream




# nav2调试

colcon build --packages-select nav2_bringup
colcon build --packages-select nav2_navfn_planner
colcon build --packages-select nav2_smac_planner
colcon build --packages-select nav_2d_utils

colcon build --packages-select nav2_costmap_2d
colcon build --packages-select nav2_dwb_controller
colcon build --packages-select dwb_core
colcon build --packages-select nav2_controller

# 楼梯检测
colcon build --packages-select go2_bringup
ros2 run go2_bringup stair_detector
ros2 run go2_bringup height_map_visualizer



# 调试步骤



colcon build --packages-select go2_bringup

ros2 launch go2_bringup go2_m_explore.launch.py 	
ros2 launch go2_bringup test_explore.launch.py 

ros2 run go2_bringup go2_move










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

1.1、调整为闭环控制？——无效
1.2 最小半径改为0——无效

1、 y轴速度——0.2可以
1、还是调大定时器间隔，——有没有定时器都差不多
2、判定为有进展的最小移动距离：从0.5调到0.15——可以
3、允许的最大无进展时间：从10秒调大5秒——可以


3、在卡死时用遥控器模拟外部控制信号——遥控器被接管，无法使用

4、保存地图——可以
