# 一、WIFI连接调试
## 安装nomachine
    1、下载安装包
    https://downloads.nomachine.com/download/?id=1

    2、远程登陆
        ssh unitree@192.168.123.18

## 开发板连接wifi下载东西

    1、用有线连接路由器，把wifi接口空出来

    2、进入wifi后台
        http://192.168.123.1/

    3、账号: admin, 密码: admin

    4、需要在导航栏“网络设置”——“WIFI 无线客户端”，勾选启用

    5、然后选择客户端无线接口模式：5.8G 客户端，搜索对应的WIFI 列表
    
    6、在SSID列表选择iphone这个热点wifi



# 三、传感器驱动安装

## 禾赛雷达
    1、启动
        ros2 launch hesai_ros_driver start.py

## realsense双目相机

    1、安装
        参考如下网址流程
        https://blog.csdn.net/qq_44998513/article/details/131517394

    2、foxy要手动安装压缩插件,以便生成压缩话题
        sudo apt install ros-foxy-image-transport ros-foxy-image-transport-plugins

    2、启动
        ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 enable_sync:=true align_depth.enable:=true

    4、卸载
        sudo apt remove ros-foxy-librealsense2
        sudo apt remove ros-foxy-realsense2-*



## L1雷达
    1、传感器数据转化
        ros2 run go2_lidar_processing transform_everything

# 三、运控

    1、AI运控
        ros2 launch go2_cmd go2_ai_cmd_test.launch.py

    2、普通运控
        ros2 run go2_cmd go2_move

## 板载机烧录
    1、连接github


# 三、建图
## slam-toolbox相关

    1、建图
        ros2 launch go2_slam_algorithm go2_slam_toolbox.launch.py 
        ros2 launch go2_lidar_processing cloud_to_scan.launch.py
        ros2 run go2_slam_algorithm motion_to_tf 
        ros2 launch go2_sim go2_urdf2tf.launch.py

    2、地图保存
    ros2 run nav2_map_server map_saver_cli -f f12-room --ros-args 
    ros2 run nav2_map_server map_saver_cli -f b1-room



## rtabmap相关

### 仿真验证
#### 2、室内2d激光和RGB-D融合slam

    SLAM:
        ros2 launch rtabmap_demos robot_mapping_demo.launch.py rviz:=true rtabmap_viz:=true

    Rosbag:
        ros2 bag play demo_mapping.db3 --clock

### 真机测试可用性
    colcon build --packages-select rtabmap_examples
    colcon build --packages-select go2_lidar_processing


    1、视觉slam
        1）在pc2上

            ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 enable_sync:=true align_depth.enable:=true

            ros2 launch rtabmap_examples realsense_d435i_color.launch.py

        
        2）在上位机上

            ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=2 enable_sync:=true align_depth.enable:=true

            ros2 launch rtabmap_examples realsense_d435i_color.launch.py

            （realsense接口的正反会有影响）


    2、激光slam

        ros2 launch hesai_ros_driver start.py

        ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=/lidar_points frame_id:=hesai_lidar 


    3、室内2d激光和RGB-D融合slam

        1）激光驱动：
            ros2 launch hesai_ros_driver start.py

            ros2 run go2_lidar_processing transform_everything

        2）双目驱动：
            ros2 launch realsense2_camera rs_launch.py
            ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30

            GUI模式：
                realsense-viewer 

        3）tf和单线激光驱动：
            ros2 launch go2_slam_algorithm go2_rtabmap_pre_pc.launch.py
            ros2 run go2_lidar_processing keep_sync


        4）rtabmap算法启动:
            ros2 launch go2_slam_algorithm robot_mapping_demo_go2.launch.py rviz:=true rtabmap_viz:=true


        colcon build --packages-select go2_lidar_processing
        colcon build --packages-select go2_slam_algorithm


        5）压缩图像检查频率
            ros2 topic hz /camera/depth/image_rect_raw
            ros2 topic hz /camera/depth/image_rect_raw/compressed
            ros2 topic hz /camera/depth/image_rect_raw/compressedDepth

            ros2 topic hz /camera/color/image_raw
            ros2 topic hz /camera/color/image_raw/compressed

    4、纯激光slam
  
            



# 导航
    1、启动nav2

        ros2 launch go2_lidar_processing cloud_to_scan.launch.py
        ros2 launch go2_nav go2_nav2_bringup.launch.py


# 三维建图
    point-lio
    1、启动建图
        colcon build --packages-select point_lio_unilidar
        ros2 launch point_lio_unilidar mapping_utlidar.launch.py

    2、启动AI云控
        ros2 launch go2_cmd go2_ai_cmd_test.launch.py

    2、查看建图
    3、查看里程计话题，尤其z轴数据


    ros2 run calibrate_imu calibrate_imu



# 自探索
## explore算法

    colcon build --packages-select explore_lite

    ros2 launch go2_explore_algorithm go2_m_explore.launch.py 


# 仿真
    ros2 launch go2_sim go2_urdf2tf.launch.py
    ros2 launch go2_sim go2_xacro2tf.launch.py
    ros2 launch go2_sim go2_gazebo_sim.launch.py
    