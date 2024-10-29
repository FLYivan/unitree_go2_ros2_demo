# 文件结构

## 主目录
setup.bash:
1、定义通信接口参数【ens37】
2、明确DDS通信模式为rmw_cyclonedds_cpp
3、source ros2的配置文件
4、source cyclonedds_ws的配置文件（狗子基于cyclonedds方法进行通信）


setup_local.sh：
1、不连接狗子，使用仿真环境时的配置文件



## /home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/src/unitree/unitree_api/
CMakeLists.txt：
1、明确ROS消息接口生成工具rosidl_generate_interfaces需要生成哪些类型接口

package.xml:
1、XML文件是一个ROS 2包的包描述文件，定义了包的基本信息、依赖关系和构建工具等

msg文件夹：
1、这个接口主要实现发送运动请求指令





## /home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/src/unitree/unitree_go/

msg文件夹：
1、这个接口定义具体状态和控制信息字段