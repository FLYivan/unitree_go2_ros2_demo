cmake_minimum_required(VERSION 3.5)  # 设置CMake最低版本要求
project(point_lio_unilidar)           # 定义项目名称

if(NOT CMAKE_BUILD_TYPE)               # 如果未设置构建类型
  set(CMAKE_BUILD_TYPE Release)        # 设置为Release模式
endif()

ADD_COMPILE_OPTIONS(-std=c++17 )       # 添加C++17编译选项
set(CMAKE_CXX_FLAGS "-std=c++17 -O3" ) # 设置C++编译标志
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions") # 设置C编译标志

set(CMAKE_CXX_STANDARD 17)             # 设置C++标准为17
set(CMAKE_CXX_STANDARD_REQUIRED ON)    # 强制要求使用该标准
set(CMAKE_CXX_EXTENSIONS OFF)          # 禁用编译器扩展
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions") # 设置其他C++编译标志

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\") # 定义根目录
message("DROOT_DIR = ${DROOT_DIR}")  # 输出根目录信息

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}") # 输出当前CPU架构
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" ) # 检查CPU架构
  include(ProcessorCount)          # 包含处理器计数模块
  ProcessorCount(N)                # 计算处理器数量
  message("Processer number:  ${N}") # 输出处理器数量
  if(N GREATER 5)                  # 如果处理器数量大于5
    add_definitions(-DMP_EN)      # 启用多处理器支持
    add_definitions(-DMP_PROC_NUM=4) # 设置处理器数量为4
    message("core for MP:  3")    # 输出多处理器核心数
  elseif(N GREATER 3)              # 如果处理器数量大于3
    math(EXPR PROC_NUM "${N} - 2") # 计算处理器数量
    add_definitions(-DMP_EN)       # 启用多处理器支持
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}") # 设置处理器数量
    message("core for MP:  ${PROC_NUM}") # 输出多处理器核心数
  else()                            # 否则
    add_definitions(-DMP_PROC_NUM=1) # 设置处理器数量为1
  endif()
else()                               # 如果不是x86架构
  add_definitions(-DMP_PROC_NUM=1)  # 设置处理器数量为1
endif()

find_package(OpenMP QUIET)          # 查找OpenMP支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # 设置C++编译标志
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}") # 设置C编译标志

find_package(PythonLibs REQUIRED)    # 查找Python库
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h") # 查找matplotlibcpp头文件

find_package(ament_cmake REQUIRED)   # 查找ament_cmake
find_package(ament_cmake_python REQUIRED) # 查找ament_cmake_python
find_package(geometry_msgs REQUIRED)  # 查找geometry_msgs
find_package(nav_msgs REQUIRED)       # 查找nav_msgs
find_package(sensor_msgs REQUIRED)    # 查找sensor_msgs
find_package(rclcpp REQUIRED)         # 查找rclcpp
find_package(rclcpp_components REQUIRED) # 查找rclcpp_components
find_package(std_msgs REQUIRED)       # 查找std_msgs
find_package(visualization_msgs REQUIRED) # 查找visualization_msgs
find_package(pcl_ros REQUIRED)        # 查找pcl_ros
find_package(tf2 REQUIRED)            # 查找tf2
find_package(tf2_geometry_msgs REQUIRED) # 查找tf2_geometry_msgs
find_package(tf2_sensor_msgs REQUIRED) # 查找tf2_sensor_msgs
find_package(tf2_eigen REQUIRED)      # 查找tf2_eigen
find_package(tf2_ros REQUIRED)        # 查找tf2_ros
find_package(Eigen3 REQUIRED)         # 查找Eigen3
find_package(PCL REQUIRED COMPONENTS common io) # 查找PCL库

find_package(Eigen3 REQUIRED)         # 再次查找Eigen3
find_package(PCL 1.8 REQUIRED)        # 查找PCL 1.8版本

message(Eigen: ${EIGEN3_INCLUDE_DIR}) # 输出Eigen的包含目录

set(dependencies                       # 设置依赖项
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  tf2_ros
  pcl_ros
)

include_directories(                  # 包含目录
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include
)

add_executable(pointlio_mapping       # 添加可执行文件
  src/laserMapping.cpp 
  include/ikd-Tree/ikd_Tree.cpp 
  src/parameters.cpp 
  src/preprocess.cpp 
  src/Estimator.cpp
)

ament_export_dependencies(rosidl_default_runtime geometry_msgs nav_msgs rclcpp rclpy std_msgs message_runtime) # 导出依赖项
ament_target_dependencies(pointlio_mapping ${dependencies}) # 设置目标依赖项
target_include_directories(pointlio_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)
target_include_directories(pointlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS}) # 设置私有包含目录
target_link_libraries(pointlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES}) # 链接库

install(TARGETS                       # 安装目标
  pointlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

install(                             # 安装目录
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()                     # 结束ament包