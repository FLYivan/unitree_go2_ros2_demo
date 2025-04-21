from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'go2_lidar_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))), 
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FLYivan',
    maintainer_email='luoyifan902008@126.com',
    description='lidar data process for go2 ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'timestamp_test  = go2_lidar_processing.timestamp_test:main', 

            'L1_lidar_switch  = go2_lidar_processing.L1_lidar_switch:main', 
            'transform_everything = go2_lidar_processing.transform_everything:main',


            'sensor_sync_node = go2_lidar_processing.sensor_sync_node:main',
        ],
    },
)
