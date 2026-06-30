#!/usr/bin/env python3
"""
aruco_detector_node
-------------------
Subscribes to a camera image + camera_info, detects ArUco marker ID=0
(DICT_4X4_50), estimates its pose with solvePnP, and publishes
/aruco/marker_pose (geometry_msgs/PoseStamped) plus an optional
/aruco/debug_image overlay.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


def _rot_to_quat(R):
    """Rotation matrix → quaternion (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_node')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.20)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('publish_debug_image', True)

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        self.pub_debug = self.get_parameter('publish_debug_image').value

        # ── ArUco detector (OpenCV ≥4.7 new API / ≤4.6 legacy API) ───────────
        # Ubuntu 20.04 / Foxy ships OpenCV 4.2–4.5 which lacks ArucoDetector,
        # while ≥4.7 removed the old free functions. Support both so the same
        # code runs on the Jetson and on dev machines.
        dict_name = self.get_parameter('aruco_dict').value
        dict_id = getattr(cv2.aruco, dict_name)
        if hasattr(cv2.aruco, 'getPredefinedDictionary'):
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        else:
            aruco_dict = cv2.aruco.Dictionary_get(dict_id)

        if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV ≥ 4.7
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, params)
            self._detect = lambda gray: detector.detectMarkers(gray)
            self._aruco_api = 'new(>=4.7)'
        else:                                            # OpenCV 4.2 – 4.6
            params = (cv2.aruco.DetectorParameters_create()
                      if hasattr(cv2.aruco, 'DetectorParameters_create')
                      else cv2.aruco.DetectorParameters())
            self._detect = lambda gray: cv2.aruco.detectMarkers(
                gray, aruco_dict, parameters=params)
            self._aruco_api = 'legacy(<=4.6)'

        # solvePnP object points for a square marker (marker frame, z=0)
        h = self.marker_size / 2.0
        self.obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

        # ── state ────────────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None

        # ── subscribers ──────────────────────────────────────────────────────
        self.create_subscription(CameraInfo, info_topic,
                                 self._info_cb, 10)
        self.create_subscription(Image, image_topic,
                                 self._image_cb, 10)

        # ── publishers ───────────────────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped,
                                              '/aruco/marker_pose', 10)
        self.debug_pub = self.create_publisher(Image,
                                               '/aruco/debug_image', 10)

        self.get_logger().info(
            f'ArUco detector ready  marker_id={self.marker_id} '
            f'size={self.marker_size}m  dict={dict_name}  '
            f'cv2={cv2.__version__} api={self._aruco_api}')

    # ── callbacks ────────────────────────────────────────────────────────────

    def _info_cb(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _image_cb(self, msg: Image):
        if self.camera_matrix is None:
            self.get_logger().warn(
                'Waiting for camera_info…', throttle_duration_sec=2.0)
            return

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self._detect(gray)

        if ids is None or self.marker_id not in ids.flatten():
            detected_ids = ids.flatten().tolist() if ids is not None else []
            self.get_logger().warn(
                f'Marker ID={self.marker_id} not detected. '
                f'Detected in frame: {detected_ids}',
                throttle_duration_sec=1.0)
            if self.pub_debug:
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
                self.debug_pub.publish(
                    self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))
            return

        idx = list(ids.flatten()).index(self.marker_id)
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, img_pts,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if not ok:
            return

        tvec = tvec.flatten()
        rvec = rvec.flatten()
        dist = float(tvec[2])
        lat = float(tvec[0])

        self.get_logger().info(
            f'Detected ID={self.marker_id}  dist={dist:.3f}m  '
            f'lateral={lat:.3f}m',
            throttle_duration_sec=0.5)

        # publish pose in camera optical frame
        R, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = _rot_to_quat(R)

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = float(tvec[0])
        ps.pose.position.y = float(tvec[1])
        ps.pose.position.z = float(tvec[2])
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

        # debug overlay
        if self.pub_debug:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            self._draw_axes(bgr, rvec, tvec, self.marker_size * 0.5)
            self.debug_pub.publish(
                self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8'))

    def _draw_axes(self, img, rvec, tvec, length):
        """Draw a pose axis overlay, compatible with old/new OpenCV."""
        if hasattr(cv2, 'drawFrameAxes'):
            cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs,
                              rvec, tvec, length)
        else:  # OpenCV < 4.2
            cv2.aruco.drawAxis(img, self.camera_matrix, self.dist_coeffs,
                               rvec, tvec, length)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
