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
            'odom2tf  = go2_bringup.odom2tf:main',   
            'frame_id_modifier  = go2_bringup.frame_id_modifier:main',   
            'motion_to_odom_node  = go2_bringup.motion_to_odom_node:main',   


        ],
    },
)
