from setuptools import setup
import os
from glob import glob

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simon Harms',
    maintainer_email='harms.simon759@mail.kyutech.jp',
    description='Detection and localization of aruco markers',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'aruco_detector = aruco_detector.aruco_detector_node:main',
        'base_pose_publisher = aruco_detector.base_pose_publisher_node:main',
        ],
    },
)
