ctrl+shirt+v 预览
# 一、GO2动作演示
## 环境配置
更改bashrc文件
测试通信192.168.123.18

## 指令接收情况
        colcon build --packages-select go2_demo_show
        ros2 run go2_demo_show brain_interface_cmd

## 摄像头拉流
        1）需要先参考CMU方法，安装组件
        2）需要opencv打开gstreamer开关
        3）用rviz2看

        colcon build --packages-select go2_camera_processing

        ros2 run go2_camera_processing video_stream --ros-args -p multicast_iface:=enp0s31f6
        ros2 run go2_camera_processing video_gui

        直接通过拉流插件看视频：
            gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=enp0s31f6 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink


gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=enp0s31f6 ! queue !  application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

## 轨迹跟踪和障碍物显示
        colcon build --packages-select go2_slam_algorithm

        启动节点
        ros2 launch go2_slam_algorithm go2_slam_toolbox.launch.py 

        启动雷达
        ros2 launch hesai_ros_driver start.py


## 脑机接口对接
        启动节点
        ros2 launch go2_demo_show brain_interface_in_go2.launch.py

        分开调试节点
        ls /dev/tty*
        ros2 run go2_demo_show brain_interface_cmd
        ros2 run go2_demo_show sport_demo

        colcon build --packages-select go2_demo_show



# 二、G1动作演示
## 环境配置
    1、更改bashrc文件，还是有线连接
    2、测试通信192.168.123.164

    3、连屏幕
        深度相机案例需要在pc2中执行代码，启动代码已放在pc2


## （一）底层运控
1、需先关闭底层电机

2、当 G1 处于悬挂状态下，并且处于阻尼状态，

3、同时按下遥控器 L2 + R2 组合键，G1 进入调试模式。

4、此时按下 L2 + A ，G1 会进入位置模式，摆出特定诊断姿势。


### C++
    在/home/flyivan/unitree/unitree_sdk2/build/bin/ 目录打开终端

    ### 下肢
    sudo ./g1_ankle_swing_example enp0s31f6     # 扭动脚踝

    ### 手臂
    sudo ./g1_dual_arm_example enp0s31f6        # 按预设轨迹移动手臂

## （二）高层控制
1、进入锁定站立模式
2、进入三自由度模式R2+X

### Python

    在/home/flyivan/unitree/unitree_sdk2_python/ 目录打开终端

    ### 下肢
        #### 运控
        python3 ./example/g1/high_level/g1_loco_client_example.py enp0s31f6

            TestOption(name="stand_up", id=1),          站起
            TestOption(name="move forward", id=3),      向前移动   
            TestOption(name="move lateral", id=4),      横向移动
            TestOption(name="move rotate", id=5),       旋转移动
            TestOption(name="low stand", id=6),         站低
            TestOption(name="high stand", id=7),        站高
            TestOption(name="wave hand1", id=9)         挥手
            TestOption(name="wave hand2", id=10)        转身挥手
            TestOption(name="shake hand", id=11)        握手

    ### 手臂
        #### 运控
        1、手臂按既定逻辑摆动
        python3 ./example/g1/high_level/g1_arm7_sdk_dds_example.py enp0s31f6

## （三）、激光雷达

    ros2 launch livox_ros_driver2 rviz_MID360_launch.py

## （四）、深度相机
    （pc2上执行）

    ros2 launch realsense2_camera rs_launch.py

  

# go2

    1、自主导航建图——地图保存
    2、非自主避障行走建图——地图保存
    3、语音交互？——惠

    4、强化学习步态展示 ——惠


    5、视频录制


    

# G1

    1、语音助手——邓
    2、灵巧手动作——邓
    3、用高层接口完成上下肢动作
    4、2d激光建图迁移（备）
    
    6、人形验证，需要人员保护