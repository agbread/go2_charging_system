# Go2 NATIVE (sport-mode) ArUco docking — BUILT-IN front camera version.
# (go2_native_docking.launch.py 기반. detector가 frontcam 전용 노드로 교체되고
#  내장 카메라 브릿지가 추가된 것만 다름. D435i 경로는 기존 launch 그대로 보존.)
#
# Brings up:
#   1. sport_mode_adapter_node   (/cmd_vel,/joy ↔ unitree SportClient;
#                                 rt/lowstate → /joint_states, /charging_state)
#   2. go2_front_camera_node     (H.264 멀티캐스트 230.1.1.1:1720, nvv4l2decoder
#                                 → /go2_front/image_raw. CameraInfo 없음)
#   3. aruco_detector_frontcam_node  (intrinsics를 calib 파일에서 로드
#                                     → /aruco/marker_pose)
#   4. aruco_docking_controller_node  (the docking state machine, unchanged)
#
# Prerequisites:
#   • Robot in sport (high-level) mode — factory walking controller active.
#   • unitree_ros2 / CycloneDDS configured for your network interface.
#   • (권장, 최초 1회) 캘리브레이션: python3 scripts/calibrate_go2_front.py
#
# Usage:
#   ros2 launch go2_sport_bridge go2_native_docking_frontcam.launch.py network_interface:=eth0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, SetEnvironmentVariable,
                            RegisterEventHandler, EmitEvent)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    docking_params = os.path.join(
        get_package_share_directory('aruco_go2_docking'),
        'config', 'docking_params_frontcam.yaml')

    net_if = LaunchConfiguration('network_interface')

    # ── Pin the right CycloneDDS core (fixes SIGSEGV/abort, exit -11) ─────────
    # go2_native_docking.launch.py와 동일한 픽스 — sport_mode_adapter_node는
    # cyclonedds_ws의 libddsc가 /usr/local 것보다 LD_LIBRARY_PATH에서 앞서야
    # 기동 시 크래시하지 않는다. 상세 설명은 원본 launch 참고.
    cdds_lib = os.path.expanduser('~/cyclonedds_ws/install/cyclonedds/lib')
    ld_fix = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        cdds_lib + ':' + os.environ.get('LD_LIBRARY_PATH', ''))

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
        parameters=[docking_params],
    )

    controller = Node(
        package='aruco_go2_docking',
        executable='aruco_docking_controller_node',
        name='aruco_docking_controller_node',
        output='screen',
        parameters=[docking_params],
    )

    # When the docking controller finishes — charging success (DONE) or full
    # failure after retries (FAILED) — it shuts itself down and its process
    # exits. Tear down the whole launch (adapter + camera + detector + this
    # launch process) so nothing lingers and you don't have to Ctrl+C / pkill.
    shutdown_on_done = RegisterEventHandler(
        OnProcessExit(
            target_action=controller,
            on_exit=[EmitEvent(event=Shutdown(reason='docking finished'))],
        )
    )

    return LaunchDescription([
        ld_fix,
        DeclareLaunchArgument('network_interface', default_value='eth0',
                              description='NIC connected to the Go2 (e.g. eth0).'),
        adapter,
        front_camera,
        detector,
        controller,
        shutdown_on_done,
    ])
