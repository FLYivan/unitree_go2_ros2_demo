from setuptools import find_packages, setup

package_name = 'pcl_process_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='LiDAR pcl process use python.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hesai_downsample_node = pcl_process_py.hesai_downsample_node:main',
        ],
    },
)
