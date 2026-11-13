"""
setup.py
────────────────────────────────────────────
Phase 1: IMU-Only Arm Mimicking System

Nodes:
  - serial_imu_bridge      : ESP32 serial → ROS2 Imu messages
  - imu_to_joint_angles_v2 : IMU quaternions → URDF joint angles
  - imu_tf_broadcaster     : Raw IMU orientation → TF frames (debug)
"""

from setuptools import setup
import os
from glob import glob

package_name = 'two_dof_arm_description'

setup(
    name=package_name,
    version='2.1.0',

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

        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='dell',
    maintainer_email='yadhukrishnan.ma.2005@gmail.com',

    description='5-DOF robotic arm URDF with dual-IMU motion mimicking',

    license='Apache-2.0',

    tests_require=['pytest'],

    # ── ROS2 executable nodes ───────────────────────────────
    entry_points={
        'console_scripts': [

            # Serial bridge: ESP32 → ROS2 Imu messages
            'serial_imu_bridge = two_dof_arm_description.serial_imu_bridge:main',

            # Kinematics: IMU quaternions → joint angles
            'imu_to_joint_angles_v2 = two_dof_arm_description.imu_to_joint_angles_v2:main',

            # TF debug: raw IMU orientations visible in RViz
            'imu_tf_broadcaster = two_dof_arm_description.imu_tf_broadcaster:main',

        ],
    },
)