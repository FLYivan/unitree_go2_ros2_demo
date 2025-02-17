# 一、本地部署步骤

    1、从 https://ollama.com/ 官网中选择linux版，按如下指令下载ollama

        curl -fsSL https://ollama.com/install.sh | sh

    2、在ollama的官网上，选择"Models"，选择deepseek-r1 32b，按如下指令下载安装

        ollama run deepseek-r1:32b
    
    3、安装openai库，以便通过api调用
        1、新建虚拟环境
        conda create --name deepseek python=3.8
        运行deepseek目录下的test.py脚本

    4、提问时，以\n开头

    

    echo $AMENT_PREFIX_PATH
/home/flyivan/human_robot/livox_lidar_ws/install/livox_ros_driver2:/home/flyivan/dog_robot/ros2_demo/lyf_demo_python/install/go2_ros2_slam:/home/flyivan/dog_robot/ros2_demo/lyf_demo_python/install/go2_ros2_basic:/home/flyivan/dog_robot/ros2_demo/lyf_demo_python/install/go2_description:/home/flyivan/dog_robot/ros2_demo/lyf_demo_python/install/go2_bringup:/home/flyivan/dog_robot/ros2_demo/lyf_demo_python/install/custom_interface:/home/flyivan/dog_robot/ros2_demo/explore/m-explore_ws/install/multirobot_map_merge:/home/flyivan/dog_robot/ros2_demo/explore/m-explore_ws/install/explore_lite:/home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/install/unitree_interfaces:/home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/install/unitree_go:/home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/install/unitree_api:/home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/install/rmw_cyclonedds_cpp:/home/flyivan/dog_robot/ros2_demo/cyclonedds_ws/install/graph_msg:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_system_tests:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_bringup:/home/flyivan/dog_robot/lib/nav2_ws/install/navigation2:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_smoother:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_graceful_controller:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_dwb_controller:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_controller:/home/flyivan/dog_robot/lib/nav2_ws/install/dwb_plugins:/home/flyivan/dog_robot/lib/nav2_ws/install/dwb_critics:/home/flyivan/dog_robot/lib/nav2_ws/install/dwb_core:/home/flyivan/dog_robot/lib/nav2_ws/install/nav_2d_utils:/home/flyivan/dog_robot/lib/nav2_ws/install/dwb_msgs:/home/flyivan/dog_robot/lib/nav2_ws/install/nav_2d_msgs:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_waypoint_follower:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_theta_star_planner:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_smac_planner:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_rotation_shim_controller:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_regulated_pure_pursuit_controller:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_planner:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_navfn_planner:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_mppi_controller:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_constrained_smoother:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_bt_navigator:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_behaviors:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_core:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_collision_monitor:/home/flyivan/dog_robot/lib/nav2_ws/install/costmap_queue:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_costmap_2d:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_voxel_grid:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_velocity_smoother:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_rviz_plugins:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_map_server:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_lifecycle_manager:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_behavior_tree:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_amcl:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_util:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_simple_commander:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_msgs:/home/flyivan/dog_robot/lib/nav2_ws/install/nav2_common:/home/flyivan/dog_robot/lib/point_to_laser_ws/install/pointcloud_to_laserscan:/home/flyivan/dog_robot/lib/slam-toolbox_ws/install/slam_toolbox:/opt/ros/humble
