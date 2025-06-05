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

## 开发板连接远程仓库
    1、远程拷贝
        scp /home/flyivan/桌面/go2_imu_calib_data.yaml unitree@192.168.123.18:/home/unitree/Desktop

    2、反向远程拷贝(仍然上位机操作)
        scp unitree@192.168.123.18:/home/unitree/dog_robot/unitree_go2_ros2_demo/unitree_go2_ros2_function_pkg/src/slam/point_lio_unilidar/PCD/scans_20250523_155726.pcd /home/flyivan/dog_robot/unitree_go2_ros2_demo/unitree_go2_ros2_function_pkg/src/slam/point_lio_unilidar/PCD/


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
        ros2 launch go2_cmd go2_ai_cmd.launch.py

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


# 三维建图point-lio(使用L1雷达)
    
    1、启动建图
        colcon build --packages-select point_lio_unilidar
        ros2 launch point_lio_unilidar mapping_utlidar.launch.py

    2、启动AI云控
        ros2 launch go2_cmd go2_ai_cmd.launch.py


# 禾赛点云降采样
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select pcl_process
    ros2 run pcl_process pcl_study
  

# 三维建图point-lio(使用禾赛雷达IMU融合)
    1、编译
        colcon build --symlink-install --packages-select transform_sensors point_lio_unilidar pcl_process
        colcon build --packages-select transform_sensors

        colcon build --symlink-install --packages-select point_lio_unilidar
        colcon build --packages-select point_lio_unilidar

        colcon build --packages-select go2_h264_repub go2_sport_api unitree_api unitree_go calibrate_imu
        colcon build --packages-select go2_lidar_processing
        colcon build --packages-select go2_cmd

    2、go2的IMU校准(在普通运控模式下)
        ros2 run go2_lidar_processing go2_imu_node
        ros2 run calibrate_imu calibrate_go2_imu

        pc2执行：
            ros2 launch go2_lidar_processing go2_imu_calibrate_on_pc2.launch.py

        上位机执行：
            ros2 run go2_lidar_processing go2_imu_node
            ros2 run calibrate_imu calibrate_go2_imu

            ros2 launch go2_lidar_processing go2_imu_calibrate.launch.py

    
    3、启动AI运控
        ros2 launch go2_cmd go2_ai_cmd.launch.py

    4、启动激光雷达
        ros2 launch hesai_ros_driver start.py

    5、传感器融合节点
        ros2 run transform_sensors transform_hesai

        ros2 run transform_sensors transform_hesai_process
        ros2 run transform_sensors sensor_fusion


    6、point-lio启动
        pc2执行：
            ros2 launch point_lio_unilidar mapping_hesai_on_go2.launch.py

        上位机执行：
            ros2 launch point_lio_unilidar mapping_hesai_lidar.launch.py

    7、节点调试
        1）pc2上地图保存
            scp unitree@192.168.123.18:/home/unitree/dog_robot/unitree_go2_ros2_demo/unitree_go2_ros2_function_pkg/src/slam/point_lio_unilidar/PCD/scans_19700218_085106.pcd /home/flyivan/dog_robot/unitree_go2_ros2_demo/unitree_go2_ros2_function_pkg/src/slam/point_lio_unilidar/PCD/

        2）节点状态查看

            ros2 topic info -v /sportmodestate
            ros2 topic info -v /utlidar/imu
            ros2 topic hz /hesai_go2/transformed_imu

        3）真实场景测试

            LAN口：
                1、全上位机执行+普通步态  可
                2、全上位机执行+AI步态   楼梯歪（未校准IMU） scans_20250527_100421  scans_20250527_101517
                3、时间戳融合pc2+slam算法上位机+AI步态 可   scans_20250527_102922
                5、全pc2+无降采样+AI步态 scans_19700209_090911 scans_19700209_141527


            WLAN口：
                1、时间戳融合pc2+slam算法上位机+AI步态
                3、全pc2+AI步态 scans_19700209_142658  scans_19700209_143842

        4）点云要做切割吗

# 自探索
## explore算法

    colcon build --packages-select explore_lite

    ros2 launch go2_explore_algorithm go2_m_explore.launch.py 


# 仿真
    ros2 launch go2_sim go2_urdf2tf.launch.py
    ros2 launch go2_sim go2_xacro2tf.launch.py
    ros2 launch go2_sim go2_gazebo_sim.launch.py


# 三维路径规划测试
    conda create --name pcl_planner python=3.10
    conda remove --name pcl_planner --all


    colcon build --packages-select pct_planner
    source install/setup.bash
    
    ros2 launch pct_planner planning.launch.py