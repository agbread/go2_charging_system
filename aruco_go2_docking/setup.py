from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'aruco_go2_docking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.world')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_board'),
         glob('models/aruco_marker_board/model.*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_board', 'materials', 'scripts'),
         glob('models/aruco_marker_board/materials/scripts/*')),
        (os.path.join('share', package_name, 'models', 'aruco_marker_board', 'materials', 'textures'),
         glob('models/aruco_marker_board/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agbread',
    maintainer_email='enbang0209@gmail.com',
    description='ArUco docking pipeline for Unitree Go2 in Gazebo',
    license='Apache-2.0',
    scripts=['scripts/calibrate_go2_front.py'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = aruco_go2_docking.aruco_detector_node:main',
            'aruco_docking_controller_node = aruco_go2_docking.aruco_docking_controller_node:main',
            'mock_charging_node = aruco_go2_docking.mock_charging_node:main',
            'go2_front_camera_node = aruco_go2_docking.go2_front_camera_node:main',
            'aruco_detector_frontcam_node = aruco_go2_docking.aruco_detector_frontcam_node:main',
        ],
    },
)
