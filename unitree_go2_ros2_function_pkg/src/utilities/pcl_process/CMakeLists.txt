cmake_minimum_required(VERSION 3.8)
project(pcl_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加编译选项
add_compile_options(-std=c++17)

# 修改这里：将可执行文件名改为 pcl_study
add_executable(pcl_study 
  src/hesai_pcl_process.cpp
)

ament_target_dependencies(pcl_study
  rclcpp 
  sensor_msgs
  pcl_conversions
)

target_link_libraries(pcl_study 
  ${PCL_LIBRARIES}
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

# 这里也要修改为 pcl_study
install(TARGETS pcl_study
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()