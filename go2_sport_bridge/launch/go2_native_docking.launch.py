# Go2 NATIVE (sport-mode) ArUco docking — no RL policy.
#
# Brings up:
#   1. sport_mode_adapter_node  (/cmd_vel,/joy ↔ unitree SportClient;
#                                rt/lowstate → /joint_states, /charging_state)
#   2. aruco_detector_node      (camera → /aruco/marker_pose)
#   3. aruco_docking_controller_node  (the docking state machine, unchanged)
#
# Prerequisites:
#   • Robot in sport (high-level) mode — factory walking controller active.
#   • unitree_ros2 / CycloneDDS configured for your network interface.
#   • RealSense driver running:  ros2 launch realsense2_camera rs_launch.py
#
# Usage:
#   ros2 launch go2_sport_bridge go2_native_docking.launch.py network_interface:=eth0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    docking_params = os.path.join(
        get_package_share_directory('aruco_go2_docking'),
        'config', 'docking_params_real.yaml')

    net_if = LaunchConfiguration('network_interface')

    adapter = Node(
        package='go2_sport_bridge',
        executable='sport_mode_adapter_node',
        name='go2_sport_mode_adapter',
        output='screen',
        parameters=[{
            'network_interface': net_if,
            # tune these to your robot; defaults are safe for docking speeds
            'max_vx': 0.6,
            'max_vy': 0.4,
            'max_vyaw': 0.8,
            'balance_stand_on_start': True,
        }],
    )

    detector = Node(
        package='aruco_go2_docking',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[docking_params],
    )

    controller = Node(
        package='aruco_go2_docking',
        executable='aruco_docking_controller_node',
        name='aruco_docking_controller_node',
        output='screen',
        parameters=[docking_params],
    )

    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value='eth0',
                              description='NIC connected to the Go2 (e.g. eth0).'),
        adapter,
        detector,
        controller,
    ])
