from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_pose_publisher'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dell',
    maintainer_email='yadhukrishnan.ma.2005@gmail.com',
    description='ArUco marker pose publisher for 5-DOF robotic arm ground truth validation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_pose_publisher.aruco_node:main',
        ],
    },
)
