"""
setup.py
────────────────────────────────────────────────
Updated for BOTH models:
• 2-DOF arm
• 5-DOF human arm

Supports:
- two_dof_arm.urdf
- human_arm_5dof.urdf
- display.launch.py
- display_5dof.launch.py
- imu_to_joint_angles
- imu_to_joint_angles_v2
"""

from setuptools import setup
import os
from glob import glob

package_name = 'two_dof_arm_description'

setup(
    name=package_name,
    version='2.0.0',   # Updated version

    packages=[package_name],

    # ── Install non-Python files ─────────────────────────────
    data_files=[

        # Register package in ROS2
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install ALL launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        # Install ALL URDF models
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.urdf'))),

        # Install RViz configurations
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='dell',
    maintainer_email='yadhukrishnan.ma.2005@gmail.com',

    description=' 5-DOF robotic arm URDF with IMU sensor integration',

    license='Apache-2.0',

    tests_require=['pytest'],

    # ── ROS2 executable nodes ───────────────────────────────
    entry_points={
        'console_scripts': [

            # Original node (2-DOF)
            'imu_to_joint_angles = two_dof_arm_description.imu_to_joint_angles:main',

            # New node for 5-DOF arm
            'imu_to_joint_angles_v2 = two_dof_arm_description.imu_to_joint_angles_v2:main',

            # Serial bridge for ESP32 raw output
            'serial_imu_bridge = two_dof_arm_description.serial_imu_bridge:main',

        ],
    },
)