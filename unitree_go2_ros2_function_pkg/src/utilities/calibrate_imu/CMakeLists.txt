cmake_minimum_required(VERSION 3.5)                                # 指定CMake最低版本要求
project(calibrate_imu)                                           # 定义项目名称

# Default to C99                                                 # C语言标准设置
if(NOT CMAKE_C_STANDARD)                                         # 如果未设置C语言标准
  set(CMAKE_C_STANDARD 99)                                       # 设置C语言标准为C99
endif()

# Default to C++14                                              # C++语言标准设置
if(NOT CMAKE_CXX_STANDARD)                                      # 如果未设置C++语言标准
  set(CMAKE_CXX_STANDARD 14)                                    # 设置C++语言标准为C++14
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  # 如果使用GCC或Clang编译器
  add_compile_options(-Wall -Wextra -Wpedantic)                 # 添加编译警告选项
endif()

include_directories(include include/nlohmann)                    # 添加头文件目录

# find dependencies                                             # 查找依赖项
find_package(ament_cmake REQUIRED)                              # 查找并导入ament_cmake包
find_package(rclcpp REQUIRED)                                   # 查找并导入rclcpp包
find_package(std_msgs REQUIRED)                                 # 查找并导入std_msgs包
find_package(sensor_msgs REQUIRED)                              # 查找并导入sensor_msgs包
find_package(geometry_msgs REQUIRED)                            # 查找并导入geometry_msgs包
find_package(unitree_api REQUIRED)                              # 查找并导入unitree_api包
find_package(go2_sport_api REQUIRED)                            # 查找并导入go2_sport_api包
find_package(unitree_go REQUIRED)                               # 查找并导入unitree_go包

# uncomment the following section in order to fill in           # 取消注释以下部分以手动添加更多依赖
# further dependencies manually.                                # 手动添加更多依赖
# find_package(<dependency> REQUIRED)                           # 查找并导入其他依赖包

# 编译 Go2 IMU 标定节点                                        # Go2 IMU标定节点编译设置(已注释)
add_executable(calibrate_go2_imu                              # 添加可执行文件
  src/calibrate_go2_imu.cpp                                  # 源文件
)
ament_target_dependencies(calibrate_go2_imu                   # 添加目标依赖
  rclcpp                                                      # ROS2 C++客户端库
  std_msgs                                                    # 标准消息库
  sensor_msgs                                                 # 传感器消息库
  geometry_msgs                                               # 几何消息库
  unitree_api                                                 # 宇树API库
  go2_sport_api                                               # Go2运动API库
  unitree_go                                                  # 宇树Go库
)



# 编译 pc2 上的Go2 IMU 标定节点                                        # Go2 IMU标定节点编译设置(已注释)
add_executable(calibrate_go2_imu_on_pc2                              # 添加可执行文件
  src/calibrate_go2_imu_on_pc2.cpp                                  # 源文件
)
ament_target_dependencies(calibrate_go2_imu_on_pc2                   # 添加目标依赖
  rclcpp                                                      # ROS2 C++客户端库
  std_msgs                                                    # 标准消息库
  sensor_msgs                                                 # 传感器消息库
  geometry_msgs                                               # 几何消息库
  unitree_api                                                 # 宇树API库
  go2_sport_api                                               # Go2运动API库
  unitree_go                                                  # 宇树Go库
)


# 编译标准 IMU 标定节点                                        # 标准IMU标定节点编译设置(已注释)
add_executable(calibrate_imu                                  # 添加可执行文件
  src/calibrate_imu.cpp                                      # 源文件
)
ament_target_dependencies(calibrate_imu                       # 添加目标依赖
  rclcpp                                                      # ROS2 C++客户端库
  std_msgs                                                    # 标准消息库
  sensor_msgs                                                 # 传感器消息库
  geometry_msgs                                               # 几何消息库
  unitree_api                                                 # 宇树API库
  go2_sport_api                                               # Go2运动API库
)

# add_executable(calibrate_imu src/calibrate_go2_imu.cpp)         # 添加IMU标定可执行文件
# ament_target_dependencies(calibrate_imu rclcpp std_msgs sensor_msgs geometry_msgs unitree_api go2_sport_api unitree_go)  # 添加目标依赖

install(TARGETS 
  calibrate_go2_imu                                                # 安装目标
  calibrate_imu                                                # 安装calibrate_imu可执行文件
  calibrate_go2_imu_on_pc2
  DESTINATION lib/${PROJECT_NAME})                             # 安装到lib目录下

# install(DIRECTORY                                            # 安装目录(已注释)
#   launch                                                     # launch文件目录
#   DESTINATION share/${PROJECT_NAME}/                         # 安装到share目录下
# )

if(BUILD_TESTING)                                              # 如果构建测试
  find_package(ament_lint_auto REQUIRED)                       # 查找并导入ament_lint_auto包
  # the following line skips the linter which checks for copyrights  # 跳过版权检查(已注释)
  # uncomment the line when a copyright and license is not present in all source files  # 当源文件缺少版权声明时取消注释
  #set(ament_cmake_copyright_FOUND TRUE)                       # 设置版权检查为已完成
  # the following line skips cpplint (only works in a git repo)  # 跳过cpplint检查(仅在git仓库中有效)
  # uncomment the line when this package is not in a git repo   # 当包不在git仓库中时取消注释
  #set(ament_cmake_cpplint_FOUND TRUE)                         # 设置cpplint检查为已完成
  ament_lint_auto_find_test_dependencies()                     # 查找测试依赖
endif()

ament_package()                                                # 声明ament包
