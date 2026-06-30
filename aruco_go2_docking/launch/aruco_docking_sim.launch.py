# Robot + environment bring-up: Gazebo (aruco_docking_test.world) + robot.
# This launch loads ONLY the robot and the simulation environment
# (no ArUco nodes). Run the ArUco detector/controller separately with:
#   ros2 launch aruco_go2_docking aruco_docking.launch.py
#
# Usage:
#   ros2 launch aruco_go2_docking aruco_docking_sim.launch.py [rname:=go2] [gui:=true]
#
# After Gazebo starts, run the RL locomotion controller in a separate terminal:
#   ros2 run rl_sar rl_sim

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (LaunchConfiguration, TextSubstitution, Command)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_aruco = get_package_share_directory('aruco_go2_docking')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    rname = LaunchConfiguration('rname')
    world_file = os.path.join(pkg_aruco, 'worlds', 'aruco_docking_test.world')
    model_path = os.path.join(pkg_aruco, 'models')

    # Extend GAZEBO_MODEL_PATH so Gazebo can find aruco_marker_board
    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=model_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', ''))

    robot_description = ParameterValue(
        Command([
            'xacro ',
            Command(['echo -n ',
                     Command(['ros2 pkg prefix ', rname, '_description'])]),
            '/share/', rname, '_description/xacro/robot.xacro',
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false',
            'world': world_file,
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'robot_model',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-Y', '0.0',
        ],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '60'],
        output='screen',
    )

    # Required by rl_sim: parameter_blackboard stores robot_name / gazebo_model_name
    robot_name_val = ParameterValue(
        Command(['echo -n ', rname]), value_type=str)
    gazebo_model_name_val = ParameterValue(
        Command(['echo -n ', rname, '_gazebo']), value_type=str)

    param_node = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='param_node',
        parameters=[{
            'robot_name': robot_name_val,
            'gazebo_model_name': gazebo_model_name_val,
        }],
    )

    # Spawn the joint_state_broadcaster only after the robot entity is in
    # Gazebo, so gazebo_ros2_control has initialized the controller_manager.
    # (imu_sensor_broadcaster intentionally omitted: go2/b2 expose no IMU via
    # ros2_control — IMU comes from libgazebo_ros_imu_sensor.so on /imu.)
    spawn_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('rname', default_value=TextSubstitution(text='b2'),
                              description='Robot name (go2 / b2 / …)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run Gazebo GUI (gzclient).'),
        set_model_path,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        spawn_controllers,
        param_node,
    ])
