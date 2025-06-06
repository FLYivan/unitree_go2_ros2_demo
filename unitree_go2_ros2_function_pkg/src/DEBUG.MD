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
ros2 launch go2_slam_algorithm go2_slam_toolbox.launch.py 
ros2 launch go2_lidar_processing cloud_to_scan.launch.py
ros2 run go2_slam_algorithm motion_to_tf 

# 地图保存
ros2 run nav2_map_server map_saver_cli -f f12-room --ros-args 
ros2 run nav2_map_server map_saver_cli -f b1-room

# 启动nav2
ros2 launch go2_lidar_processing cloud_to_scan.launch.py
ros2 launch go2_nav go2_nav2_bringup.launch.py


# AI运控

    ros2 launch go2_cmd go2_ai_cmd.launch.py

# 启动explore节点并同步建图
   ## go2
   
    # 拓展乌
    ros2 launch hesai_ros_driver start.py

    # 上位机
    ros2 launch go2_explore_algorithm go2_m_explore.launch.py 



# 启动仿真
    
    ros2 launch go2_sim go2_urdf2tf.launch.py
    ros2 launch go2_sim go2_xacro2tf.launch.py
    ros2 launch go2_sim go2_gazebo_sim.launch.py
    

    
# L1雷达点云转化

    ros2 run go2_lidar_processing transform_everything
    ros2 launch go2_lidar_processing cloud_to_scan_L1.launch.py


# 优化explore算法
colcon build --packages-select explore_lite

# 运动轨迹显示
colcon build --packages-select go2_slam_algorithm
ros2 run go2_slam_algorithm trajectory_visualizer

# 摄像头拉流驱动
ros2 run go2_camera_processing video_stream --ros-args --remap multicast_iface:=wlp0s20f3


# 楼梯检测
colcon build --packages-select go2_bringup
ros2 run go2_slam_algorithm height_map_visualizer



# 宇树slam服务
    1、远程拷贝
        scp GO2_SLAM20250401.zip unitree@192.168.123.18:/home/unitree/Desktop