from setuptools import find_packages, setup
import os

package_name = 'go2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), ['launch/cloud_to_scan.py']),
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

        ],
    },
)
