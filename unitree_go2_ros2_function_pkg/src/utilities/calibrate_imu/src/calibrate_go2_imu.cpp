#include <iostream>                                    // 包含输入输出流头文件
#include <chrono>                                     // 包含时间相关头文件
#include <vector>                                     // 包含向量容器头文件
#include <fstream>                                    // 包含文件流头文件
#include <unistd.h>                                   // 包含POSIX操作系统API头文件

#include "rclcpp/rclcpp.hpp"                         // 包含ROS2 C++客户端库头文件
#include <sensor_msgs/msg/imu.hpp>                   // 包含IMU消息类型头文件
#include <geometry_msgs/msg/twist_stamped.hpp>       // 包含带时间戳的速度消息类型头文件

#include "unitree_api/msg/request.hpp"               // 包含宇树机器人API请求消息头文件
#include "common/ros2_sport_client.h"                // 包含运动控制客户端头文件

int state = -1;                                      // 定义状态变量，初始化为-1

std::vector<sensor_msgs::msg::Imu> imu_static;                      // 存储静止状态下的IMU数据
std::vector<sensor_msgs::msg::Imu> imu_rotation_positive_z;         // 存储正向Z轴旋转时的IMU数据
// std::vector<sensor_msgs::msg::Imu> imu_rotation_negative_z;      // 存储负向Z轴旋转时的IMU数据（已注释）

double acc_bias_x = 0;                               // 加速度计X轴偏差
double acc_bias_y = 0;                               // 加速度计Y轴偏差
double acc_bias_z = 0;                               // 加速度计Z轴偏差

double ang_bias_x = 0;                               // 陀螺仪X轴偏差
double ang_bias_y = 0;                               // 陀螺仪Y轴偏差
double ang_bias_z = 0;                               // 陀螺仪Z轴偏差

double ang_z2x_proj = 0;                             // Z轴到X轴的投影系数
double ang_z2y_proj = 0;                             // Z轴到Y轴的投影系数

void imu_handler(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in)    // IMU数据处理回调函数
{
    // double theta = 15.1 / 180 * 3.1415926;          // 将角度转换为弧度，调整为Z轴向上  
    //                                                 // 将角度转换为弧度，调整为Z轴向上
    //                                                 // 15.1度是IMU的初始安装角度，3.1415926是π的值

    double x = msg_in->angular_velocity.x;           // 获取原始角速度X分量
    double y = msg_in->angular_velocity.y;          // 获取原始角速度Y分量并取反
    double z = msg_in->angular_velocity.z;          // 获取原始角速度Z分量并取反

    // double x2 = x * cos(theta) - z * sin(theta);     // 计算旋转后的角速度X分量
    // double y2 = y;                                   // 角速度Y分量保持不变
    // double z2 = x * sin(theta) + z * cos(theta);     // 计算旋转后的角速度Z分量

    double acc_x = msg_in->linear_acceleration.x;    // 获取原始加速度X分量
    double acc_y = msg_in->linear_acceleration.y;   // 获取原始加速度Y分量并取反
    double acc_z = msg_in->linear_acceleration.z;   // 获取原始加速度Z分量并取反

    // double acc_x2 = acc_x * cos(theta) - acc_z * sin(theta);   // 计算旋转后的加速度X分量
    // double acc_y2 = acc_y;                                      // 加速度Y分量保持不变
    // double acc_z2 = acc_x * sin(theta) + acc_z * cos(theta);   // 计算旋转后的加速度Z分量

    sensor_msgs::msg::Imu msg_store = *msg_in;      // 创建新的IMU消息用于存储

    msg_store.angular_velocity.x = x;               // 存储转换后的角速度X分量
    msg_store.angular_velocity.y = y;               // 存储转换后的角速度Y分量
    msg_store.angular_velocity.z = z;               // 存储转换后的角速度Z分量
    msg_store.linear_acceleration.x = acc_x;        // 存储转换后的加速度X分量
    msg_store.linear_acceleration.y = acc_y;        // 存储转换后的加速度Y分量
    msg_store.linear_acceleration.z = acc_z;        // 存储转换后的加速度Z分量
    
    if (state == 1){                                // 如果处于静止状态
        imu_static.push_back(msg_store);            // 将数据存入静止数据向量
    }
    else if (state == 2){                          // 如果处于正向旋转状态
        imu_rotation_positive_z.push_back(msg_store);  // 将数据存入正向旋转数据向量
    }
}

void serialize_to_file(){                           // 将标定结果序列化到文件的函数
    const char* homeDir = getenv("HOME");          // 获取用户主目录
    std::string file_path = std::string(homeDir) + "/桌面/go2_imu_calib_data.yaml";  // 构建文件路径
    std::ofstream file;                            // 创建输出文件流
    file.open(file_path, std::ios::out);          // 打开文件
    file << "acc_bias_x: " << acc_bias_x << std::endl;    // 写入加速度计X轴偏差
    file << "acc_bias_y: " << acc_bias_y << std::endl;    // 写入加速度计Y轴偏差
    file << "acc_bias_z: " << acc_bias_z << std::endl;    // 写入加速度计Z轴偏差
    file << "ang_bias_x: " << ang_bias_x << std::endl;    // 写入陀螺仪X轴偏差
    file << "ang_bias_y: " << ang_bias_y << std::endl;    // 写入陀螺仪Y轴偏差
    file << "ang_bias_z: " << ang_bias_z << std::endl;    // 写入陀螺仪Z轴偏差
    file << "ang_z2x_proj: " << ang_z2x_proj << std::endl;  // 写入Z轴到X轴的投影系数
    file << "ang_z2y_proj: " << ang_z2y_proj << std::endl;  // 写入Z轴到Y轴的投影系数
    file.close();                                  // 关闭文件
}

void estimate_bias(){                              // 估计偏差的函数
    // 从静止数据中估计常量偏差
    // 处理静止状态数据计算偏差：
    //      计算加速度计三轴偏差
    //      计算陀螺仪三轴偏差
    //      虑重力加速度影响
    // 处理旋转状态数据计算投影系数：
    //      计算Z轴旋转时X/Y轴的投影关系
    int static_count = 0;                         // 静止数据计数器
    int positive_rot_count = 0;                   // 正向旋转数据计数器
    int negative_rot_count = 0;                   // 负向旋转数据计数器

    double ang_rot_x_mean_positive = 0;           // 正向旋转时X轴角速度平均值
    double ang_rot_y_mean_positive = 0;           // 正向旋转时Y轴角速度平均值
    double ang_rot_z_mean_positive = 0;           // 正向旋转时Z轴角速度平均值

    double ang_rot_x_mean_negative = 0;           // 负向旋转时X轴角速度平均值
    double ang_rot_y_mean_negative = 0;           // 负向旋转时Y轴角速度平均值
    double ang_rot_z_mean_negative = 0;           // 负向旋转时Z轴角速度平均值
    
    std::cout << "static size:" << imu_static.size() << std::endl;                // 输出静止数据数量
    std::cout << "pos size:" << imu_rotation_positive_z.size() << std::endl;      // 输出正向旋转数据数量
    // std::cout << "neg size:" << imu_rotation_negative_z.size() << std::endl;   // 输出负向旋转数据数量（已注释）

    for (auto imu_data: imu_static){              // 遍历静止状态数据
        static_count += 1;                        // 计数器增加

        acc_bias_x += imu_data.linear_acceleration.x;    // 累加加速度X分量
        acc_bias_y += imu_data.linear_acceleration.y;    // 累加加速度Y分量
        acc_bias_z += imu_data.linear_acceleration.z;    // 累加加速度Z分量

        ang_bias_x += imu_data.angular_velocity.x;       // 累加角速度X分量
        ang_bias_y += imu_data.angular_velocity.y;       // 累加角速度Y分量
        ang_bias_z += imu_data.angular_velocity.z;       // 累加角速度Z分量
    }
    acc_bias_x /= imu_static.size();             // 计算加速度X轴偏差平均值
    acc_bias_y /= imu_static.size();             // 计算加速度Y轴偏差平均值
    acc_bias_z /= imu_static.size();             // 计算加速度Z轴偏差平均值
    ang_bias_x /= imu_static.size();             // 计算角速度X轴偏差平均值
    ang_bias_y /= imu_static.size();             // 计算角速度Y轴偏差平均值
    ang_bias_z /= imu_static.size();             // 计算角速度Z轴偏差平均值
    
    acc_bias_z -= 9.81;                          // 减去重力加速度

    std::cout << "acc_bias_x: " << acc_bias_x << std::endl;    // 输出加速度X轴偏差
    std::cout << "acc_bias_y: " << acc_bias_y << std::endl;    // 输出加速度Y轴偏差
    std::cout << "acc_bias_z: " << acc_bias_z << std::endl;    // 输出加速度Z轴偏差
    std::cout << "ang_bias_x: " << ang_bias_x << std::endl;    // 输出角速度X轴偏差
    std::cout << "ang_bias_y: " << ang_bias_y << std::endl;    // 输出角速度Y轴偏差
    std::cout << "ang_bias_z: " << ang_bias_z << std::endl;    // 输出角速度Z轴偏差

    for (auto imu_data: imu_rotation_positive_z){        // 遍历正向旋转数据
        positive_rot_count += 1;                         // 计数器增加
        ang_rot_x_mean_positive += imu_data.angular_velocity.x;   // 累加X轴角速度
        ang_rot_y_mean_positive += imu_data.angular_velocity.y;   // 累加Y轴角速度
        ang_rot_z_mean_positive += imu_data.angular_velocity.z;   // 累加Z轴角速度
    }
    ang_rot_x_mean_positive = ang_rot_x_mean_positive / imu_rotation_positive_z.size() - ang_bias_x;    // 计算去偏差后的X轴角速度平均值
    ang_rot_y_mean_positive = ang_rot_y_mean_positive / imu_rotation_positive_z.size() - ang_bias_y;    // 计算去偏差后的Y轴角速度平均值
    ang_rot_z_mean_positive = ang_rot_z_mean_positive / imu_rotation_positive_z.size() - ang_bias_z;    // 计算去偏差后的Z轴角速度平均值

    double positive_comp_x = -ang_rot_x_mean_positive / ang_rot_z_mean_positive;    // 计算X轴补偿系数
    double positive_comp_y = -ang_rot_y_mean_positive / ang_rot_z_mean_positive;    // 计算Y轴补偿系数
    std::cout << "positive_comp_x: " << positive_comp_x << std::endl;               // 输出X轴补偿系数
    std::cout << "positive_comp_y: " << positive_comp_y << std::endl;               // 输出Y轴补偿系数
    
    ang_z2x_proj = positive_comp_x;              // 保存Z轴到X轴的投影系数
    ang_z2y_proj = positive_comp_y;              // 保存Z轴到Y轴的投影系数
}

/**
 * 状态机：
 * 0-2: 调整初始位置 (0)
 * 2-15: 纯静止状态用于标定偏差 (1)，从3秒开始收集数据
 * 15-35: z轴旋转: 80 度/秒 (2)
 * 35-37: 写入文件
 * 退出 
*/
int main(int argc, char** argv)                   // 主函数
{
    rclcpp::init(argc, argv);                    // 初始化ROS2
    auto nh = rclcpp::Node::make_shared("calibrate_go2_imu");    // 创建节点

    auto pubGo2Request = nh->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);    // 创建运动请求发布者
    auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 5);            // 创建速度命令发布者（用于调试）

    auto subImu = nh->create_subscription<sensor_msgs::msg::Imu>("/go2/imu", 300,  imu_handler);  // 创建IMU数据订阅者

    geometry_msgs::msg::TwistStamped cmd_vel;    // 创建速度命令消息
    cmd_vel.header.frame_id = "vehicle";         // 设置速度命令的坐标系

    rclcpp::Rate rate(100);                      // 设置循环频率为100Hz
    bool status = rclcpp::ok();                  // 获取ROS2运行状态
    auto beginning = std::chrono::system_clock::now();    // 记录开始时间

    unitree_api::msg::Request req;               // 创建运动请求消息
    SportClient sport_req;                       // 创建运动控制客户端

    bool file_written = false;                   // 文件写入标志
    
    double ang_vel = 1.396;                      // 设置角速度值

    while (status){                              // 主循环
        rclcpp::spin_some(nh);                   // 处理ROS2回调
        
        auto current = std::chrono::system_clock::now();    // 获取当前时间
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(current - beginning).count();    // 计算运行时间

        if (seconds < 2){                        // 前2秒：调整初始位置
            cmd_vel.twist.linear.x = 0;          // 设置线速度X为0
            cmd_vel.twist.linear.y = 0;          // 设置线速度Y为0
            cmd_vel.twist.angular.z = 0;         // 设置角速度Z为0
            pubSpeed->publish(cmd_vel);          // 发布速度命令

            sport_req.Move(req, 0, 0, 0);        // 发送停止运动请求
            pubGo2Request->publish(req);         // 发布运动请求
            
            state = 0;                           // 设置状态为0
        }
        else if (seconds >= 2 && seconds <15){   // 2-15秒：静止状态收集数据
            cmd_vel.twist.linear.x = 0;          // 设置线速度X为0
            cmd_vel.twist.linear.y = 0;          // 设置线速度Y为0
            cmd_vel.twist.angular.z = 0;         // 设置角速度Z为0
            pubSpeed->publish(cmd_vel);          // 发布速度命令

            sport_req.StopMove(req);             // 发送停止运动请求
            pubGo2Request->publish(req);         // 发布运动请求

            if (seconds >= 5){                   // 5秒后开始收集数据
                state = 1;                       // 设置状态为1
            }
        }
        else if (seconds >= 15 && seconds < 35){ // 15-35秒：执行Z轴正向旋转
            cmd_vel.twist.linear.x = 0;          // 设置线速度X为0
            cmd_vel.twist.linear.y = 0;          // 设置线速度Y为0
            cmd_vel.twist.angular.z = ang_vel;   // 设置角速度Z为预定值
            pubSpeed->publish(cmd_vel);          // 发布速度命令

            sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z);    // 发送运动请求
            pubGo2Request->publish(req);         // 发布运动请求

            state = 2;                           // 设置状态为2
        }
        else if (seconds >= 35 && seconds < 37){ // 35-37秒：停止运动
            cmd_vel.twist.linear.x = 0;          // 设置线速度X为0
            cmd_vel.twist.linear.y = 0;          // 设置线速度Y为0
            cmd_vel.twist.angular.z = 0;         // 设置角速度Z为0
            pubSpeed->publish(cmd_vel);          // 发布速度命令

            sport_req.StopMove(req);             // 发送停止运动请求
            pubGo2Request->publish(req);         // 发布运动请求

            state = 3;                           // 设置状态为3
        }

        if (state == 0){                         // 状态0：输出调整提示
            std::cout << "Adjusting the robot to the initial position..." << std::endl;    // 输出正在调整位置信息
        }
        else if (state == 1){                    // 状态1：输出数据收集提示
            std::cout << "Collecting static data..." << std::endl;    // 输出正在收集静态数据信息
        }
        else if (state == 2){                    // 状态2：输出旋转数据收集提示
            std::cout << "Collecting positive z-axis rotation data..." << std::endl;    // 输出正在收集旋转数据信息
        }
        else if (state == 3){                    // 状态3：输出文件写入提示
            std::cout << "Writing to file..." << std::endl;    // 输出正在写入文件信息
        }
        if (state == 3 && !file_written){        // 状态3且文件未写入：执行数据处理和保存
            estimate_bias();                      // 估计偏差
            serialize_to_file();                  // 序列化到文件
            file_written = true;                 // 设置文件已写入标志
            std::cout << "Calibration finished!" << std::endl;    // 输出标定完成信息
            break;                               // 退出循环
        }
        status = rclcpp::ok();                  // 更新ROS2运行状态
        rate.sleep();                           // 循环延时
    }
}
