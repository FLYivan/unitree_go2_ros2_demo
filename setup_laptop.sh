#!/bin/bash
echo "Setup unitree ros2 python environment"
# source /opt/ros/humble/setup.bash
# source $HOME/dog_robot/unitree/unitree_ros2/cyclonedds_ws/install/setup.bash
source $HOME/dog_robot/ros2_demo/cyclonedds_ws/install/local_setup.bash
source $HOME/dog_robot/ros2_demo/explore/m-explore_ws/install/local_setup.bash
source $HOME/dog_robot/ros2_demo/lyf_demo_python/install/local_setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp0s31f6" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
