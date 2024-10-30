from setuptools import find_packages, setup
import os

package_name = 'go2_ros2_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

         # 安装launch文件和YAML文件
        (os.path.join('share', package_name, 'launch'), ['launch/slam_launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/unitree_slam_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flyivan',
    maintainer_email='luoyifan902008@126.com',
    description='go2 slam algorithm package in ROS2 by python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slam_node  = go2_ros2_slam.go2_slam:main',   
            'unitree_slam_service  = go2_ros2_slam.unitree_slam:main',  
            'slam_node_test  = go2_ros2_slam.go2_slam_test:main',                              
        ],
    },
)
