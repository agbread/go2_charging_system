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
