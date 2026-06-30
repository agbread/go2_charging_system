# Real Go2 — ArUco docking nodes only.
# Prerequisites (run before this launch):
#   1. rl_sar real Go2:  ros2 run rl_sar rl_real_go2_ros
#   2. RealSense driver: ros2 launch realsense2_camera rs_launch.py
#
# Usage:
#   ros2 launch aruco_go2_docking aruco_docking_real.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('aruco_go2_docking'),
        'config', 'docking_params_real.yaml')

    detector = Node(
        package='aruco_go2_docking',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[params],
    )

    controller = Node(
        package='aruco_go2_docking',
        executable='aruco_docking_controller_node',
        name='aruco_docking_controller_node',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([detector, controller])
