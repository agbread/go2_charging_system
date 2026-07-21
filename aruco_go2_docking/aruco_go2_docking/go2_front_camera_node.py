#!/usr/bin/env python3
"""
go2_front_camera_node
----------------------
Bridges the Unitree Go2 built-in front camera (H.264 over UDP multicast,
230.1.1.1:1720, hardware-decoded via nvv4l2decoder) into ROS.

Publishes /go2_front/image_raw (bgr8) ONLY — no CameraInfo. Intrinsics are
not this node's business: aruco_detector_frontcam_node loads them directly
from the calibration file (~/ros2_ws/src/go2_front_calib.yaml) at startup.

frame_id: go2_front_camera. Actual measured delivery rate is ~14.4 fps —
matches the datasheet's 15 fps. (The H.264 caps advertise framerate=30/1,
but that is container metadata only; buffer-count measurement on the live
stream shows ~14.4. Don't trust the caps number.)
"""

import array
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image

from aruco_go2_docking.go2_front_gst_receiver import Go2FrontGstReceiver

# If the stream produces no new frame for this long, restart the GStreamer
# pipeline (handles the Go2 dropping/re-establishing the multicast stream).
STALE_FRAME_RESTART_SEC = 5.0
WATCHDOG_PERIOD_SEC = 1.0


class Go2FrontCameraNode(Node):

    def __init__(self):
        super().__init__('go2_front_camera_node')

        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('frame_id', 'go2_front_camera')

        self.network_interface = self.get_parameter('network_interface').value
        self.frame_id = self.get_parameter('frame_id').value

        self.image_pub = self.create_publisher(Image, '/go2_front/image_raw', 10)

        self._last_frame_time = None

        self.receiver = Go2FrontGstReceiver(
            network_interface=self.network_interface,
            on_frame=self._on_frame,
            on_error=self._on_gst_error,
        )
        self.receiver.start()
        decoder = ('nvv4l2decoder(HW)' if self.receiver.using_hw_decoder
                   else 'avdec_h264(SW fallback)')
        self.get_logger().info(
            f'Go2 front camera bridge ready — iface={self.network_interface} '
            f'multicast=230.1.1.1:1720 decoder={decoder} → /go2_front/image_raw')
        if not self.receiver.using_hw_decoder:
            self.get_logger().warn(
                '하드웨어 디코더(nvv4l2decoder) 초기화 실패 — avdec_h264(소프트웨어)로 동작. '
                'Jetson이 아니거나 NVDEC 초기화 문제. 성능 저하 가능.')

        self.create_timer(WATCHDOG_PERIOD_SEC, self._watchdog_tick)

    # ── frame handling (called from the GStreamer thread) ─────────────────────

    def _on_frame(self, frame_bgr, width, height):
        self._last_frame_time = time.monotonic()

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = height
        msg.width = width
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = width * 3
        # 주의: `msg.data = bytes`는 금지 — Foxy rosidl의 uint8[] setter가 bytes를
        # 원소 단위로 검증해 1280x720 프레임 기준 ~550ms 걸림 (실측). array.array는
        # setter가 fast-path로 통과시켜 ~0.5ms. 이거 하나로 발행률이 3.6→14.4Hz 됨.
        data = array.array('B')
        data.frombytes(frame_bgr.tobytes())
        msg.data = data
        self.image_pub.publish(msg)

    # ── stream health / reconnect ──────────────────────────────────────────

    def _on_gst_error(self, msg):
        self.get_logger().error(f'GStreamer 오류: {msg}')

    def _watchdog_tick(self):
        if self._last_frame_time is None:
            self.get_logger().warn(
                '수신 프레임 없음 — 멀티캐스트 스트림 대기 중 '
                f'(iface={self.network_interface}). 로봇 내부망 연결 확인.',
                throttle_duration_sec=5.0)
            return
        stale = time.monotonic() - self._last_frame_time
        if stale > STALE_FRAME_RESTART_SEC:
            self.get_logger().warn(
                f'{stale:.1f}초간 프레임 없음 — GStreamer 파이프라인 재연결 시도.')
            self.receiver.restart()
            self._last_frame_time = time.monotonic()

    def destroy_node(self):
        self.receiver.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Go2FrontCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
