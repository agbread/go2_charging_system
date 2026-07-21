#!/usr/bin/env python3
"""
go2_front_live_view — 내장 전면 카메라 브라우저 라이브 뷰어 (보조 도구)
------------------------------------------------------------------------
헤드리스 환경에서 카메라 화면을 실시간으로 보기 위한 MJPEG HTTP 서버.
캘리브레이션(calibrate_go2_front.py) 중에 같이 띄워두고, 같은 네트워크의
노트북/휴대폰 브라우저로 접속해서 체커보드가 화면 어디에 있는지 확인하는 용도.

사용법:
    python3 go2_front_live_view.py            # 포트 8080
    → 브라우저에서 http://<이 젯슨의 IP>:8080  (실행 시 주소 출력됨)

멀티캐스트는 다중 수신이 가능하므로 캘리브레이션 스크립트와 동시 실행해도
서로 방해하지 않는다 (nvv4l2decoder 다중 세션도 Orin에서 동작 확인됨).
"""

import argparse
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import cv2

from aruco_go2_docking.go2_front_gst_receiver import Go2FrontGstReceiver

PAGE = """<!DOCTYPE html>
<html><head><title>Go2 front camera</title>
<style>body{margin:0;background:#111;display:flex;flex-direction:column;
align-items:center;font-family:sans-serif;color:#ccc}
img{max-width:100vw;height:auto}</style></head>
<body><p>Go2 내장 전면 카메라 — 실시간</p>
<img src="/stream"></body></html>
"""


def local_ips():
    ips = []
    try:
        for _, _, _, _, sockaddr in socket.getaddrinfo(socket.gethostname(), None,
                                                       socket.AF_INET):
            if not sockaddr[0].startswith('127.'):
                ips.append(sockaddr[0])
    except socket.gaierror:
        pass
    # getaddrinfo가 빈손이면 인터페이스 IP를 직접 수집
    if not ips:
        import subprocess
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
        ips = [ip for ip in out.stdout.split() if not ip.startswith('127.')]
    return ips or ['<이 장비 IP>']


class MjpegHandler(BaseHTTPRequestHandler):
    receiver = None       # set in main()
    fps = 10.0
    quality = 80

    def log_message(self, fmt, *args):
        pass  # 접속 로그 소음 제거

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            body = PAGE.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path != '/stream':
            self.send_response(404)
            self.end_headers()
            return

        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        period = 1.0 / self.fps
        last_ts = None
        try:
            while True:
                frame, ts = self.receiver.get_latest_frame()
                if frame is None or ts == last_ts:
                    time.sleep(0.03)
                    continue
                last_ts = ts
                ok, jpg = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, self.quality])
                if not ok:
                    continue
                self.wfile.write(b'--frame\r\n')
                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                self.wfile.write(f'Content-Length: {len(jpg)}\r\n\r\n'.encode())
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b'\r\n')
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass  # 브라우저 탭 닫힘 — 정상 종료


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--iface', default='eth0', help='Go2 내부망 NIC (기본: eth0)')
    ap.add_argument('--port', type=int, default=8080, help='HTTP 포트 (기본: 8080)')
    ap.add_argument('--fps', type=float, default=10.0, help='스트림 fps 상한 (기본: 10)')
    ap.add_argument('--quality', type=int, default=80, help='JPEG 품질 1-100 (기본: 80)')
    args = ap.parse_args()

    receiver = Go2FrontGstReceiver(network_interface=args.iface)
    receiver.start()
    print(f'디코더: {"nvv4l2decoder(HW)" if receiver.using_hw_decoder else "avdec_h264(SW)"}')

    MjpegHandler.receiver = receiver
    MjpegHandler.fps = args.fps
    MjpegHandler.quality = args.quality

    server = ThreadingHTTPServer(('0.0.0.0', args.port), MjpegHandler)
    print('브라우저에서 접속:')
    for ip in local_ips():
        print(f'  http://{ip}:{args.port}')
    print('(같은 네트워크에 있는 기기에서. 종료: Ctrl+C)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n종료.')
    finally:
        server.shutdown()
        receiver.stop()


if __name__ == '__main__':
    main()
