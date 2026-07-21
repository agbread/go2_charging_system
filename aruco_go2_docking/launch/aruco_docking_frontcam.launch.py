# Real Go2 — ArUco docking with the BUILT-IN front camera, RL locomotion.
# (aruco_docking_real.launch.py 기반. detector가 frontcam 전용 노드로 교체되고
#  내장 카메라 브릿지가 추가된 것만 다름. D435i 경로는 기존 launch 그대로 보존.)
#
# Prerequisites (run before this launch):
#   1. rl_sar real Go2:  ros2 run rl_sar rl_real_go2_ros
#   2. (권장, 최초 1회) 캘리브레이션: python3 scripts/calibrate_go2_front.py
#      — 없으면 detector가 스펙 근사 intrinsics로 동작 (WARN)
#
# Usage:
#   ros2 launch aruco_go2_docking aruco_docking_frontcam.launch.py network_interface:=eth0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('aruco_go2_docking'),
        'config', 'docking_params_frontcam.yaml')

    net_if = LaunchConfiguration('network_interface')

    front_camera = Node(
        package='aruco_go2_docking',
        executable='go2_front_camera_node',
        name='go2_front_camera_node',
        output='screen',
        parameters=[{'network_interface': net_if}],
    )

    detector = Node(
        package='aruco_go2_docking',
        executable='aruco_detector_frontcam_node',
        name='aruco_detector_frontcam_node',
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

    return LaunchDescription([
        DeclareLaunchArgument('network_interface', default_value='eth0',
                              description='Go2 내부망(멀티캐스트 수신) NIC.'),
        front_camera,
        detector,
        controller,
    ])
