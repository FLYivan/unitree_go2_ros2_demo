from setuptools import find_packages, setup

package_name = 'go2_lidar_processing'

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
    description='lidar data process for go2 ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [

            'timestamp_test  = go2_lidar_processing.timestamp_test:main', 
        ],
    },
)
