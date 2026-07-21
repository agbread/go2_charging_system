#!/usr/bin/env python3
"""
go2_front_calib_io
-------------------
Shared load/save for the Go2 front-camera calibration YAML
(default ~/ros2_ws/src/go2_front_calib.yaml), plus the spec-sheet fallback intrinsics
used when no calibration file exists yet.

Producers/consumers:
  scripts/calibrate_go2_front.py       — writes the file (run once, offline)
  aruco_detector_frontcam_node.py      — reads it once at startup
(No CameraInfo topic anywhere in the frontcam path.)

File format:
    image_width: int
    image_height: int
    camera_matrix: [9 floats]         # row-major 3x3: fx,0,cx, 0,fy,cy, 0,0,1
    distortion_coefficients: [5 floats]
    distortion_model: plumb_bob
    rms: float
"""

import os
import shutil

import yaml
import numpy as np

# Unitree Go2 built-in front camera: 1280x720, HFOV 100 deg, VFOV 56 deg.
# fx = 640/tan(50deg) ~= 537.0, fy = 360/tan(28deg) ~= 677.0.
# fx != fy is an artifact of approximating a wide-FOV lens with a pinhole
# model from FOV alone — undistorted only near the image center. Real
# accuracy comes from running scripts/calibrate_go2_front.py once.
SPEC_IMAGE_WIDTH = 1280
SPEC_IMAGE_HEIGHT = 720
SPEC_FX = 537.0
SPEC_FY = 677.0
SPEC_CX = 640.0
SPEC_CY = 360.0
DISTORTION_MODEL = 'plumb_bob'

DEFAULT_CALIB_PATH = '~/ros2_ws/src/go2_front_calib.yaml'


def spec_fallback():
    """Spec-sheet-approximated calibration dict (used when no calib file)."""
    return {
        'image_width': SPEC_IMAGE_WIDTH,
        'image_height': SPEC_IMAGE_HEIGHT,
        'camera_matrix': [SPEC_FX, 0.0, SPEC_CX,
                           0.0, SPEC_FY, SPEC_CY,
                           0.0, 0.0, 1.0],
        'distortion_coefficients': [0.0, 0.0, 0.0, 0.0, 0.0],
        'distortion_model': DISTORTION_MODEL,
        'rms': None,
    }


def load_calib(path):
    """Loads a calibration YAML. Returns the dict, or None if the file does
    not exist. Raises ValueError if the file exists but is malformed."""
    path = os.path.expanduser(path)
    if not os.path.isfile(path):
        return None
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    required = ('image_width', 'image_height', 'camera_matrix',
                'distortion_coefficients')
    missing = [k for k in required if k not in data]
    if missing:
        raise ValueError(f'{path}: missing keys {missing}')
    if len(data['camera_matrix']) != 9:
        raise ValueError(f'{path}: camera_matrix must have 9 elements')
    data.setdefault('distortion_model', DISTORTION_MODEL)
    data.setdefault('rms', None)
    return data


def save_calib(path, image_width, image_height, camera_matrix, dist_coeffs,
               rms, distortion_model=DISTORTION_MODEL):
    """Saves a calibration YAML, backing up any existing file to path+'.bak'."""
    path = os.path.expanduser(path)
    camera_matrix = np.asarray(camera_matrix, dtype=float).reshape(-1).tolist()
    dist_coeffs = np.asarray(dist_coeffs, dtype=float).reshape(-1).tolist()
    if len(camera_matrix) != 9:
        raise ValueError('camera_matrix must have 9 elements')

    if os.path.isfile(path):
        shutil.copy2(path, path + '.bak')

    data = {
        'image_width': int(image_width),
        'image_height': int(image_height),
        'camera_matrix': camera_matrix,
        'distortion_coefficients': dist_coeffs,
        'distortion_model': distortion_model,
        'rms': float(rms),
    }
    with open(path, 'w') as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    return data


def as_camera_matrix_np(calib_dict):
    return np.array(calib_dict['camera_matrix'], dtype=np.float64).reshape(3, 3)


def as_dist_coeffs_np(calib_dict):
    return np.array(calib_dict['distortion_coefficients'], dtype=np.float64)
