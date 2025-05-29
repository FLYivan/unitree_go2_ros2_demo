# 安装依赖库
  sudo apt update
  sudo apt install ros-humble-slam-toolbox
  

# 编译

  1、在 unitree_go2_ros2_function_pkg 目录下执行如下命令
  2、colcon build

# 设置环境变量
  将如下命令添加到.bashrc文件
    source 安装目录/unitree_go2_ros2_demo/setup_laptop.sh
  打开新终端

# 运行
  ros2 launch go2_demo_show brain_interface_in_go2.launch.py
