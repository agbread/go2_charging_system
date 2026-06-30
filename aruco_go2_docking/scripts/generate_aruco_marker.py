#!/usr/bin/env python3
"""
Generate ArUco marker PNG for Gazebo texture.

Run once before the first simulation launch:
    python3 scripts/generate_aruco_marker.py

Options:
    --id      Marker ID (default 0)
    --size    Output image size in pixels (default 600)
    --mirror  Flip horizontally (use if the texture appears mirrored in Gazebo)
    --output  Custom output path
"""

import argparse
import os
import sys

import cv2
import numpy as np


def generate(marker_id: int = 0, size_px: int = 600,
             mirror: bool = False, output_path: str = None):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    inner = int(size_px * 512 / 600)
    pad = (size_px - inner) // 2

    marker_img = np.zeros((inner, inner), dtype=np.uint8)
    cv2.aruco.generateImageMarker(aruco_dict, marker_id, inner, marker_img, 1)

    canvas = np.ones((size_px, size_px), dtype=np.uint8) * 255
    canvas[pad:pad + inner, pad:pad + inner] = marker_img

    if mirror:
        canvas = cv2.flip(canvas, 1)

    bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)

    if output_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(
            script_dir, '..', 'models', 'aruco_marker_board',
            'materials', 'textures', 'aruco_0.png')

    output_path = os.path.abspath(output_path)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, bgr)
    print(f'Saved: {output_path}  ({size_px}x{size_px}px, mirror={mirror})')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate ArUco marker PNG')
    parser.add_argument('--id', type=int, default=0, help='Marker ID')
    parser.add_argument('--size', type=int, default=600,
                        help='Output image size in pixels')
    parser.add_argument('--mirror', action='store_true',
                        help='Flip horizontally (if texture appears mirrored)')
    parser.add_argument('--output', type=str, default=None)
    args = parser.parse_args()
    generate(args.id, args.size, args.mirror, args.output)
