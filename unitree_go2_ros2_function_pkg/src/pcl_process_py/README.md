# SLAM LiDAR Odometry

This repository implements LiDAR-based odometry using the Iterative Closest Point (ICP) algorithm for pose estimation. It is designed to process sequential LiDAR point clouds and estimate motion as part of a SLAM (Simultaneous Localization and Mapping) pipeline.

## Features
- Point-to-plane ICP implementation for precise frame-to-frame motion estimation.
- ROS2 node integration for real-time LiDAR odometry.
- Modular and extensible design for SLAM pipelines.

## Requirements
- Python 3.8+
- ROS2 (Humble or later)
- Required Python libraries:
  - `numpy`
  - `scipy`
  - `open3d`
 
## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/dariusfratila/slam-lidar-odometry.git
   cd slam-lidar-odometry

2. Build the ROS2 package & source the ROS2 workspace:
   ```bash
   colcon build
   source install/setup.bash

3. Launch the LiDAR odometry node:
   ```bash
   ros2 run icp_lidar_odometry lidar_odometry
   
