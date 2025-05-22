cmake_minimum_required(VERSION 3.8)
project(pcl_process)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_process 
  src/hesai_pcl_process.cpp
)
ament_target_dependencies(pcl_process 
  rclcpp 
  sensor_msgs
)

target_link_libraries(pcl_process ${PCL_LIBRARIES})

ament_package()






