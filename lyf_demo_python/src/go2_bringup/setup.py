from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'go2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
    maintainer='flyivan',
    maintainer_email='luoyifan902008@126.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frame_id_modifier  = go2_bringup.frame_id_modifier:main',   
            'go2_move  = go2_bringup.go2_move:main',  
            'initialpose_publisher  = go2_bringup.initialpose_publisher:main',  
            'motion_to_tf  = go2_bringup.motion_to_tf:main',  
            'stair_detector  = go2_bringup.stair_detector:main',   
            'trajectory_visualizer'= go2_bringup.trajectory_visualizer:main', 

            'wavefront_frontier  = go2_bringup.wavefront_frontier:main', 

            'timestamp_test  = go2_bringup.timestamp_test:main', 
            'find_stairs  = go2_bringup.find_stairs:main',     
                     
            'height_map_visualizer  = go2_bringup.height_map_visualizer:main',            


        ],
    },
)
