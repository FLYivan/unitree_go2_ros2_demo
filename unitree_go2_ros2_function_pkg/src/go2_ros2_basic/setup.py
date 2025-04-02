from setuptools import find_packages, setup

package_name = 'go2_ros2_basic'

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
    maintainer='flyivan',
    maintainer_email='luoyifan902008@126.com',
    description='go2 control&state in ROS2 by python',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dog_state_sub  = go2_ros2_basic.read_motion_state:main',
            'dog_control_pub  = go2_ros2_basic.sport_mode_ctrl:main',
            'sport_demo  = go2_ros2_basic.sport_demo:main',


        ],
    },
)
