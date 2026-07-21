#!/usr/bin/env python3
"""
aruco_detector_frontcam_node
-----------------------------
aruco_detector_node.py 복사본 — 차이는 intrinsics를 CameraInfo 토픽 대신
calib 파일(~/ros2_ws/src/go2_front_calib.yaml)에서 시작 시 1회 로드하는 것뿐.
원본(aruco_detector_node.py) 수정 시 여기도 반영할 것.

Go2 내장 전면 카메라는 CameraInfo를 제공하지 않으므로, 이 노드는
`calib_file` 파라미터의 YAML(scripts/calibrate_go2_front.py 산출물)에서
camera_matrix/dist_coeffs를 직접 읽는다. 파일이 없으면 스펙 근사값으로
동작한다 (WARN — 중앙부만 유효, 캘리브레이션 권장).

검출/solvePnP//aruco/marker_pose 발행/debug image는 원본과 동일.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

from aruco_go2_docking import go2_front_calib_io as calib_io


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


class ArucoDetectorFrontcamNode(Node):

    def __init__(self):
        super().__init__('aruco_detector_frontcam_node')

        # ── parameters ──────────────────────────────────────────────────────
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size', 0.20)
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('image_topic', '/go2_camera/image_raw')
        self.declare_parameter('calib_file', calib_io.DEFAULT_CALIB_PATH)
        self.declare_parameter('publish_debug_image', True)

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size').value
        image_topic = self.get_parameter('image_topic').value
        calib_file = self.get_parameter('calib_file').value
        self.pub_debug = self.get_parameter('publish_debug_image').value

        # ── intrinsics: calib 파일에서 시작 시 1회 로드 (CameraInfo 미사용) ────
        self.camera_matrix, self.dist_coeffs = self._load_intrinsics(calib_file)

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

        # ── subscribers ──────────────────────────────────────────────────────
        self.create_subscription(Image, image_topic,
                                 self._image_cb, 10)

        # ── publishers ───────────────────────────────────────────────────────
        self.pose_pub = self.create_publisher(PoseStamped,
                                              '/aruco/marker_pose', 10)
        self.debug_pub = self.create_publisher(Image,
                                               '/aruco/debug_image', 10)

        self.get_logger().info(
            f'ArUco frontcam detector ready  marker_id={self.marker_id} '
            f'size={self.marker_size}m  dict={dict_name}  '
            f'cv2={cv2.__version__} api={self._aruco_api}')

    def _load_intrinsics(self, calib_file):
        try:
            calib = calib_io.load_calib(calib_file)
        except Exception as e:
            self.get_logger().error(
                f'캘리브레이션 파일 로드 실패 ({e}) — 근사치로 대체.')
            calib = None

        if calib is not None:
            rms = calib.get('rms')
            rms_txt = f'{rms:.2f}' if rms is not None else '?'
            self.get_logger().info(
                f'캘리브레이션 로드됨: {calib_file} (rms={rms_txt})')
        else:
            self.get_logger().warn(
                '캘리브레이션 파일 없음 — 근사치 동작. '
                'calibrate_go2_front.py 실행 권장')
            calib = calib_io.spec_fallback()

        return calib_io.as_camera_matrix_np(calib), calib_io.as_dist_coeffs_np(calib)

    # ── callbacks ────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
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
    node = ArucoDetectorFrontcamNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
