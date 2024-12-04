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
ros2 launch go2_bringup clou
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

ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
ros2 launch explore_lite explore.launch.py

