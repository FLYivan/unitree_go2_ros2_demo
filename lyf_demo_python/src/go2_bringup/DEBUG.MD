# 通用代码
ros2 pkg create --license Apache-2.0 --build-type ament_python go2_ros2_basic

ros2 run rqt_tf_tree rqt_tf_tree --force-discover

colcon build --packages-select go2_ros2_slam
colcon build --packages-select go2_description
colcon build --packages-select go2_bringup

ros2 run go2_bringup motion_to_odom_node 

ros2 run go2_ros2_slam slam_node
ros2 run go2_bringup motion_to_tf 
ros2 run go2_bringup change_initialpose 
ros2 run go2_ros2_basic sport_demo 

# 建图
ros2 launch go2_bringup go2_slam_bringup.launch.py 
ros2 launch go2_bringup cloud_to_scan.launch.py
ros2 launch go2_bringup slam.launch.py 

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


ros2 launch nav2_bringup tb3_simulation_launch.py slam:=True
ros2 launch explore_lite explore.launch.py

