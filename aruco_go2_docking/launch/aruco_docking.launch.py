# Simulation nodes-only launch — assumes Gazebo is already running.
# Includes mock_charging_node to simulate /charging_state in Gazebo.
#
# Usage:
#   ros2 launch aruco_go2_docking aruco_docking.launch.py
#
# Toggle mock charging result at runtime:
#   ros2 param set /mock_charging_node charging_success true   # → success
#   ros2 param set /mock_charging_node charging_success false  # → fail (default)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('aruco_go2_docking'),
        'config', 'docking_params.yaml')

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

    mock_charging = Node(
        package='aruco_go2_docking',
        executable='mock_charging_node',
        name='mock_charging_node',
        output='screen',
        parameters=[{'charging_success': False}],
    )

    return LaunchDescription([detector, controller, mock_charging])
