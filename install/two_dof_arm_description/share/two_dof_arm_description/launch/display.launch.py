"""
launch/display_5dof.launch.py

ROS2 Launch file to visualize the 5-DOF human-like arm in RViz.

Usage:
    ros2 launch two_dof_arm_description display_5dof.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Launch arguments
    use_gui_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='false',  # Set to 'false' to disable GUI sliders by default
        description='Whether to launch joint_state_publisher_gui'
    )
    use_gui = LaunchConfiguration('use_gui')

    # Get package path
    pkg_share = get_package_share_directory('two_dof_arm_description')

    # Path to URDF
    urdf_file = os.path.join(pkg_share, 'urdf', 'human_arm_5dof.urdf')

    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # ── Node 1: robot_state_publisher ─────────────────────────────
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # ── Node 2: joint_state_publisher (conditional GUI) ───────────
    # When use_gui is true
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(use_gui),
        output='screen'
    )

    # ── Node 3: RViz2 ─────────────────────────────────────────────
    rviz_config = os.path.join(pkg_share, 'rviz', 'arm_view.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        use_gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])