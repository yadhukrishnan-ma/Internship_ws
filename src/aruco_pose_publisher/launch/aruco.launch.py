"""
launch/aruco.launch.py

Launch the ArUco Pose Publisher node with parameters from YAML config.

Usage:
    ros2 launch aruco_pose_publisher aruco.launch.py
    ros2 launch aruco_pose_publisher aruco.launch.py show_preview:=false
    ros2 launch aruco_pose_publisher aruco.launch.py camera_matrix_path:=/path/to/camera_matrix.npy
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('aruco_pose_publisher')
    params_file = os.path.join(pkg_share, 'config', 'aruco_params.yaml')

    # ── Launch arguments for runtime override ─────────────────
    show_preview_arg = DeclareLaunchArgument(
        'show_preview',
        default_value='true',
        description='Show OpenCV preview window with annotated markers'
    )

    camera_matrix_arg = DeclareLaunchArgument(
        'camera_matrix_path',
        default_value='',
        description='Path to camera_matrix.npy (leave empty for hardcoded values)'
    )

    dist_coeffs_arg = DeclareLaunchArgument(
        'dist_coeffs_path',
        default_value='',
        description='Path to dist_coeffs.npy (leave empty for hardcoded values)'
    )

    # ── ArUco Pose Publisher Node ─────────────────────────────
    aruco_node = Node(
        package='aruco_pose_publisher',
        executable='aruco_node',
        name='aruco_pose_publisher',
        output='screen',
        parameters=[
            params_file,
            {
                'show_preview': LaunchConfiguration('show_preview'),
                'camera_matrix_path': LaunchConfiguration('camera_matrix_path'),
                'dist_coeffs_path': LaunchConfiguration('dist_coeffs_path'),
            }
        ],
    )

    return LaunchDescription([
        show_preview_arg,
        camera_matrix_arg,
        dist_coeffs_arg,
        aruco_node,
    ])
