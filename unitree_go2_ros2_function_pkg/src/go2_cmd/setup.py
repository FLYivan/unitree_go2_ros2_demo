from setuptools import find_packages, setup

package_name = 'go2_cmd'

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
    description='go2 basic contrl',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'go2_move  = go2_cmd.go2_move:main',  
            'go2_obstacle avoidance  = go2_cmd.go2_obstacle avoidance:main',   
        ],
    },
)
