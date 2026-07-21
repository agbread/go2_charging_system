#!/usr/bin/env python3
"""
calibrate_go2_front — Go2 내장 전면 카메라 캘리브레이션 (단독 실행 도구)
------------------------------------------------------------------------
launch에 포함되지 않는 독립 스크립트. 매 실행마다가 아니라 필요할 때(최초
1회, 로봇 교체 시) 돌려서 ~/ros2_ws/src/go2_front_calib.yaml 을 갱신하는 용도.
aruco_detector_frontcam_node 가 시작할 때 이 파일을 읽으므로, 저장 후
detector 를 재시작하면 반영 끝 — 별도 절차 없음.

헤드리스(SSH) 전용: 화면 없이 터미널 텍스트 안내만으로 진행.
카메라 수신부는 브릿지 노드와 동일 모듈(go2_front_gst_receiver) 공유.

사용법:
    # 캘리브레이션 (기본: 10x7 내부코너, 25mm 칸 — Mark Hedley Jones A4 보드)
    python3 calibrate_go2_front.py
    python3 calibrate_go2_front.py --pattern 10x7 --square 0.0245 --num-frames 40

    # 스냅샷: 프레임 1장만 저장하고 종료 (이미지 방향/품질 눈확인용, scp로 회수)
    python3 calibrate_go2_front.py --snapshot /tmp/front.jpg

인쇄물 주의: 100% 배율(실제 크기)로 인쇄하고, 자로 4칸=100mm 검증.
다르면 실측 칸 길이를 --square 로 입력 (예: 24.5mm → 0.0245).
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import cv2
import numpy as np

from aruco_go2_docking.go2_front_gst_receiver import Go2FrontGstReceiver
from aruco_go2_docking import go2_front_calib_io as calib_io

# 수집 조건 3가지 (전부 만족해야 1장 수집):
#  ① 정지 상태: 직전 검출 프레임과 거의 같은 위치 → 보드를 멈춰 든 상태 (모션블러 방지)
#  ② 새 포즈: 이미 수집된 "모든" 프레임과 충분히 다른 위치/크기 (직전 하나가 아니라 전체와
#     비교 — 연속으로 움직이기만 해도 매 프레임 수집되던 문제 방지)
#  ③ 최소 간격: 직전 수집 후 일정 시간 경과 (옮길 시간 확보)
STILL_MAX_SHIFT_PX = 12.0        # ① 이내면 "멈춰 있음"
STILL_MAX_SCALE_RATIO = 0.04
MIN_CENTROID_SHIFT_PX = 40.0     # ② 기존 수집분 전체와 이 이상 달라야 새 포즈
MIN_SCALE_CHANGE_RATIO = 0.12
MIN_COLLECT_INTERVAL_SEC = 3.0   # ③

NO_FRAME_TIMEOUT_SEC = 10.0

TIPS = [
    '보드를 화면 중앙에 정면으로 보여주세요',
    '보드를 화면 왼쪽 가장자리로 옮겨보세요',
    '보드를 화면 오른쪽 가장자리로 옮겨보세요',
    '보드를 화면 위쪽 가장자리로 옮겨보세요',
    '보드를 화면 아래쪽 가장자리로 옮겨보세요',
    '보드를 좌우로 기울여보세요 (roll)',
    '보드를 상하로 기울여보세요 (pitch/yaw)',
    '카메라에 더 가까이 대보세요',
    '카메라에서 더 멀리 대보세요',
    '보드를 코너 쪽(모서리)으로 옮겨보세요',
]


def _corner_metric(corners):
    """(centroid_x, centroid_y, span) — 포즈 다양성 판정용."""
    pts = corners.reshape(-1, 2)
    centroid = pts.mean(axis=0)
    span = np.linalg.norm(pts[0] - pts[-1])
    return float(centroid[0]), float(centroid[1]), float(span)


def _shift_and_scale(a, b):
    shift = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
    scale = abs(a[2] - b[2]) / max(b[2], 1e-6)
    return shift, scale


def _is_still(curr, prev):
    """직전 검출 프레임과 거의 같은 자리 → 보드를 멈춰 들고 있다."""
    if prev is None:
        return False
    shift, scale = _shift_and_scale(curr, prev)
    return shift <= STILL_MAX_SHIFT_PX and scale <= STILL_MAX_SCALE_RATIO


def _is_novel(curr, collected):
    """이미 수집된 모든 포즈와 충분히 다르다."""
    for m in collected:
        shift, scale = _shift_and_scale(curr, m)
        if shift < MIN_CENTROID_SHIFT_PX and scale < MIN_SCALE_CHANGE_RATIO:
            return False
    return True


def _wait_first_frame(receiver, iface):
    """스트림에서 첫 프레임을 기다린다. 실패 시 안내 후 None."""
    deadline = time.monotonic() + NO_FRAME_TIMEOUT_SEC
    while time.monotonic() < deadline:
        frame, ts = receiver.get_latest_frame()
        if frame is not None:
            return frame
        time.sleep(0.1)
    print(f'[오류] {NO_FRAME_TIMEOUT_SEC:.0f}초간 프레임을 받지 못했습니다.')
    print(f'  확인: 로봇 내부망(--iface {iface}) 연결, 멀티캐스트 라우팅.')
    print('  수신 자체 검증: gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 '
          f'multicast-iface={iface} ! application/x-rtp, media=video, '
          'encoding-name=H264 ! rtph264depay ! h264parse ! nvv4l2decoder ! fakesink')
    return None


def run_snapshot(receiver, path, iface):
    frame = _wait_first_frame(receiver, iface)
    if frame is None:
        return 1
    path = os.path.expanduser(path)
    cv2.imwrite(path, frame)
    h, w = frame.shape[:2]
    print(f'스냅샷 저장: {path} ({w}x{h})')
    print('scp로 가져가서 확인하세요 — 상하좌우 방향이 정상인지(뒤집힘이면 '
          'lateral/heading 부호가 반전됨), 디코드 깨짐/색 이상이 없는지.')
    return 0


def run_calibration(receiver, args):
    try:
        cols, rows = (int(v) for v in args.pattern.lower().split('x'))
    except ValueError:
        print(f'[오류] --pattern 형식이 잘못됨: {args.pattern} (예: 10x7)')
        return 1
    pattern_size = (cols, rows)

    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square

    print('Go2 전면 카메라 캘리브레이션')
    print(f'  체커보드: {cols}x{rows} 내부코너, 한 칸 {args.square * 1000:.1f}mm')
    print(f'  목표 프레임 수: {args.num_frames}')
    print(f'  저장 경로: {args.output}')
    print()

    if _wait_first_frame(receiver, args.iface) is None:
        return 1
    print('스트림 수신 확인. 체커보드를 카메라에 보여주세요.\n')

    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                  | cv2.CALIB_CB_FAST_CHECK)

    obj_points, img_points = [], []
    collected_metrics = []
    prev_metric = None          # 직전 "검출" 프레임 (수집 여부 무관) — 정지 판정용
    image_size = None
    last_ts_seen = None
    last_collect_time = 0.0
    tip_idx = 0

    try:
        while len(img_points) < args.num_frames:
            frame, ts = receiver.get_latest_frame()
            if frame is None or ts == last_ts_seen:
                time.sleep(0.05)
                continue
            last_ts_seen = ts
            image_size = (frame.shape[1], frame.shape[0])

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size,
                                                       flags=find_flags)
            if not found:
                prev_metric = None
                continue

            metric = _corner_metric(corners)
            still = _is_still(metric, prev_metric)
            prev_metric = metric

            # ① 멈춰 있고 ② 새 포즈이고 ③ 직전 수집에서 시간이 지났을 때만 수집
            if not still:
                continue
            if not _is_novel(metric, collected_metrics):
                continue
            if time.monotonic() - last_collect_time < MIN_COLLECT_INTERVAL_SEC:
                continue

            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                       subpix_criteria)
            img_points.append(corners)
            obj_points.append(objp.copy())
            collected_metrics.append(metric)
            last_collect_time = time.monotonic()

            print(f'수집 {len(img_points)}/{args.num_frames} — '
                  f'{TIPS[tip_idx % len(TIPS)]}')
            tip_idx += 1

    except KeyboardInterrupt:
        print('\n중단됨 (Ctrl+C).')
        if len(img_points) < 10:
            print(f'수집 {len(img_points)}장뿐 — 캘리브레이션 생략 (최소 10장 권장).')
            return 1
        print(f'수집된 {len(img_points)}장으로 진행합니다.')

    print(f'\n{len(img_points)}장으로 calibrateCamera 실행 중...')
    rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
        obj_points, img_points, image_size, None, None)

    print(f'RMS 재투영 오차: {rms:.3f}px')
    if rms < 1.0:
        print('  → 양호')
    elif rms < 2.0:
        print('  → 사용 가능, 재시도 권장 (더 다양한 각도/거리로 재수집하면 개선 여지)')
    else:
        print('  → 재수집 필요. 보드 평탄도/조명/인쇄 배율(100%) 확인 후 재시도.')
        # TODO: RMS가 계속 크면 cv2.fisheye.calibrate()로 어안 모델 재시도 옵션
        # 추가 가능 (HFOV 100° 광각이라 plumb_bob이 가장자리에서 부족할 수 있음).

    calib_io.save_calib(args.output,
                        image_width=image_size[0], image_height=image_size[1],
                        camera_matrix=camera_matrix, dist_coeffs=dist_coeffs,
                        rms=rms)
    print(f'\n저장 완료: {os.path.expanduser(args.output)} (기존 파일은 .bak 백업)')
    print('aruco_detector_frontcam_node를 재시작하면 자동 반영됩니다.')
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--iface', '--network-interface', dest='iface', default='eth0',
                    help='Go2 내부망 NIC (기본: eth0)')
    ap.add_argument('--pattern', default='10x7',
                    help='체커보드 내부코너 "가로x세로" (기본: 10x7)')
    ap.add_argument('--square', type=float, default=0.025,
                    help='체커보드 한 칸 길이 [m] (기본: 0.025). 인쇄물 실측값 입력')
    ap.add_argument('--num-frames', type=int, default=30,
                    help='수집 목표 프레임 수 (기본: 30, 권장 30~40)')
    ap.add_argument('--output', default=calib_io.DEFAULT_CALIB_PATH,
                    help=f'저장 경로 (기본: {calib_io.DEFAULT_CALIB_PATH})')
    ap.add_argument('--snapshot', metavar='PATH', default=None,
                    help='캘리브레이션 없이 프레임 1장만 PATH에 저장하고 종료')
    args = ap.parse_args()

    receiver = Go2FrontGstReceiver(network_interface=args.iface)
    receiver.start()
    print(f'디코더: {"nvv4l2decoder(HW)" if receiver.using_hw_decoder else "avdec_h264(SW fallback)"}')
    try:
        if args.snapshot:
            return run_snapshot(receiver, args.snapshot, args.iface)
        return run_calibration(receiver, args)
    finally:
        receiver.stop()


if __name__ == '__main__':
    sys.exit(main())
