from setuptools import find_packages, setup

package_name = 'go2_slam_algorithm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FLYivan',
    maintainer_email='luoyifan902008@126.com',
    description='some slam algorithm can be used in go2 ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'trajectory_visualizer = go2_slam_algorithm.trajectory_visualizer:main', 
            'motion_to_tf  = go2_slam_algorithm.motion_to_tf:main',  
            'height_map_visualizer  = go2_slam_algorithm.height_map_visualizer:main',  
            'frame_id_modifier  = go2_slam_algorithm.frame_id_modifier:main',  
        ],
    },
)
