from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'go2_cmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name],
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
    description='go2 basic contrl',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'go2_move  = go2_cmd.go2_move:main',  
            'go2_obstacle_avoidance  = go2_cmd.go2_obstacle_avoidance:main',   

            'go2_p2r_cmd  = go2_cmd.go2_p2r_cmd:main',  
            'go2_ai_cmd  = go2_cmd.go2_ai_cmd:main',  

            'vel_pub_test  = go2_cmd.vel_pub_test:main',  
            'cmd_pub_test  = go2_cmd.cmd_pub_test:main', 

        ],
    },
)
