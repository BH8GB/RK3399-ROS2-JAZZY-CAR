from glob import glob
import os

from setuptools import setup

package_name = 'n10_slam_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='2D SLAM bringup for the Leishen N10 serial lidar.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'base_serial_node = n10_slam_bringup.base_serial_node:main',
            'mecanum_keyboard = n10_slam_bringup.mecanum_keyboard_node:main',
        ],
    },
)
