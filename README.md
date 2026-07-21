# go2_charging_system

Unitree **Go2**의 ArUco 마커 기반 **자율 충전(도킹) 시스템**.
카메라로 충전 패드의 ArUco 마커를 인식해 접근·정렬한 뒤 패드 위에 엎드려 충전하고,
결과를 `/aruco_state` 토픽으로 상위 애플리케이션에 알립니다.

```
                 [카메라] ──► aruco_detector_node ──► /aruco/marker_pose
                                                            │
                                                            ▼
 /aruco_state ◄── aruco_docking_controller_node (도킹 상태머신)
 (상위 앱 알림)             │ /cmd_vel · /joy
                            ▼
      ┌── 시뮬: rl_sar (RL 보행 정책)
      └── 실제 로봇: go2_sport_bridge ──► unitree_sdk2 SportClient (내장 sport-mode 제어기)
                     ▲
                rt/lowstate ──► /joint_states · /charging_state (BMS)
```

## 패키지 구성

| 패키지 | 언어 | 역할 |
|---|---|---|
| `aruco_go2_docking` | Python | 도킹 "두뇌": ArUco 마커 검출 + 도킹 상태머신(접근→정렬→앉기→충전 확인→재시도) + 시뮬용 mock 충전 노드 + **내장 전면 카메라 지원**(H.264 멀티캐스트 브릿지, 전용 detector, 캘리브레이션 도구) |
| [`go2_sport_bridge`](go2_sport_bridge/README.md) | C++ | 실제 로봇 어댑터: `/cmd_vel`·`/joy` ↔ SportClient 번역, `rt/lowstate` → `/joint_states`·`/charging_state` |

```
aruco_go2_docking/
├── aruco_go2_docking/
│   ├── aruco_detector_node.py          # ArUco 마커 감지 → /aruco/marker_pose (D435i, CameraInfo 토픽)
│   ├── aruco_detector_frontcam_node.py # 〃 내장 전면 카메라용 — intrinsics를 calib 파일에서 로드
│   ├── aruco_docking_controller_node.py # 도킹 FSM + /aruco_state 발행
│   ├── mock_charging_node.py           # 시뮬용: /charging_state 모의 발행
│   ├── go2_front_camera_node.py        # 내장 카메라 브릿지: H.264 멀티캐스트 → /go2_front/image_raw
│   ├── go2_front_gst_receiver.py       # 공용 GStreamer 수신 모듈 (nvv4l2decoder HW 디코드)
│   └── go2_front_calib_io.py           # 캘리브레이션 yaml 로드/저장 공용 모듈
├── config/
│   ├── docking_params.yaml             # 시뮬레이션 파라미터
│   ├── docking_params_real.yaml        # 실제 로봇 파라미터 (D435i)
│   └── docking_params_frontcam.yaml    # 실제 로봇 파라미터 (내장 전면 카메라, RL/sport 공용)
├── launch/
│   ├── aruco_docking.launch.py         # 시뮬: 노드만 (Gazebo 별도 실행 후)
│   ├── aruco_docking_sim.launch.py     # 시뮬: Gazebo 환경 구성
│   ├── aruco_docking_real.launch.py    # 실제 로봇 전용 (D435i + RL)
│   └── aruco_docking_frontcam.launch.py # 실제 로봇 전용 (내장 카메라 + RL)
├── models/aruco_marker_board/          # Gazebo 마커 모델
├── scripts/
│   ├── generate_aruco_marker.py        # 마커 PNG 생성
│   ├── calibrate_go2_front.py          # 내장 카메라 캘리브레이션 (단독 실행, 헤드리스)
│   └── go2_front_live_view.py          # 내장 카메라 브라우저 라이브 뷰어 (MJPEG)
└── worlds/aruco_docking_test.world

go2_sport_bridge/
├── src/sport_mode_adapter_node.cpp     # /cmd_vel·/joy ↔ SportClient 어댑터
└── launch/
    ├── go2_native_docking.launch.py          # 실제 로봇 일괄 실행 (D435i + sport)
    └── go2_native_docking_frontcam.launch.py # 실제 로봇 일괄 실행 (내장 카메라 + sport)

go2_front_calib.yaml                    # 내장 카메라 캘리브레이션 결과 (RMS 0.689px)
```

## 빠른 실행

### 시뮬레이션 (Gazebo)

터미널 3개를 열고 순서대로 실행합니다:

```bash
# Gazebo 환경 + 로봇
ros2 launch aruco_go2_docking aruco_docking_sim.launch.py rname:=go2
# RL 로코모션 (rl_sar — 아래 참고)
ros2 run rl_sar rl_sim
# 도킹 노드 + mock 충전
ros2 launch aruco_go2_docking aruco_docking.launch.py
# 충전 성공 시뮬레이션
ros2 param set /mock_charging_node charging_success true
```

> `rl_sar`(RL 보행 컨트롤러)와 `go2_description`은 이 저장소에 포함되어 있지 않습니다.
> [RCILab/RCI_quadruped_robot_navigation](https://github.com/RCILab/RCI_quadruped_robot_navigation)
> 저장소를 같은 워크스페이스(`~/ros2_ws/src`)에 받아 함께 빌드한 뒤 실행하면 됩니다.

### 실제 로봇 (Go2 내장 sport-mode 제어기)

```bash
# RealSense 카메라
ros2 launch realsense2_camera rs_launch.py
# 도킹 전체 (어댑터 + detector + controller) — 완료 시 자동 종료
ros2 launch go2_sport_bridge go2_native_docking.launch.py network_interface:=eth0
```
> ※ 실제 로봇은 CycloneDDS 환경설정(`RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`)이 필수입니다.
> 누락 시 어댑터가 `Failed to create domain explicitly`로 즉시 죽습니다 — [ONBOARDING.md](ONBOARDING.md) 4장 참고.

### 실제 로봇 (내장 전면 카메라 — RealSense 불필요)

```bash
# 최초 1회: 캘리브레이션 (FRONTCAM_GUIDE.md 참고)
python3 ~/ros2_ws/src/aruco_go2_docking/scripts/calibrate_go2_front.py
# 도킹 전체 (어댑터 + 카메라 브릿지 + frontcam detector + controller) — 완료 시 자동 종료
ros2 launch go2_sport_bridge go2_native_docking_frontcam.launch.py network_interface:=eth0
```
> 캘리브레이션·검증 절차·파라미터 실측값은 [FRONTCAM_GUIDE.md](FRONTCAM_GUIDE.md) 참고.

### 실제 로봇 (강화학습 제어기)

```bash
# RealSense 카메라
ros2 launch realsense2_camera rs_launch.py
# RL 로코모션 (rl_sar)
ros2 run rl_sar rl_real_go2 eth0
# 도킹 전체 (detector + controller) — 완료 시 자동 종료
ros2 launch aruco_go2_docking aruco_docking_real.launch.py 
```
> `rl_sar`(RL 보행 컨트롤러)와 `go2_description`은 이 저장소에 포함되어 있지 않습니다.
> [fan-ziqi/rl_sar](https://github.com/fan-ziqi/rl_sar)
> 저장소를 Go2 로봇에 받아 빌드한 뒤 실행하면 됩니다. 자세한 제어기 실행 방법은 저장소를 참고하면 됩니다.

## `/aruco_state` 상태 알림 (연동 규격)

| 값 | 의미 | 이후 동작 |
|---|---|---|
| `aruco_arrive` | 충전 패드 위에 엎드림 확인 | 충전 확인 대기 |
| `aruco_success` | 약 10초간 충전 상태 유지 성공 | 서비스 자동 종료 |
| `aruco_failed` | 총 3회 시도 모두 실패 | 일어선 뒤 자동 종료 |

## 문서

| 문서 | 내용 |
|---|---|
| **[RUN_GUIDE.md](RUN_GUIDE.md)** | **전달용 통합 실행 가이드** — 실행 파일 목록·순서, 파일별 역할, 환경/의존성, 파라미터/옵션 |
| **[FRONTCAM_GUIDE.md](FRONTCAM_GUIDE.md)** | **내장 전면 카메라 도킹 가이드** — 카메라 확인(스냅샷/라이브뷰어/토픽), 캘리브레이션 실행법과 양호 판정 기준(RMS<1.0px), 적용 확인, 1m 거리 오차 검증(marker_size 역산), target_distance 확정, 도킹 실행, 실측 확정 파라미터 |
| [ONBOARDING.md](ONBOARDING.md) | 새 PC 설치 절차 (의존성 설치, CycloneDDS/DDS 설정, 트러블슈팅) |
| [go2_sport_bridge/README.md](go2_sport_bridge/README.md) | sport-mode 어댑터 상세 (번역 테이블, 보정 포인트) |
| [연동 규격서 PDF](aruco_go2_docking/Unitree%20Go2%20자율%20충전%20상태%20알림%20연동%20규격서.pdf) | `/aruco_state` 연동 규격 원문 |

## 검증 환경

Ubuntu 20.04 (Jetson) · ROS 2 Foxy · CycloneDDS 0.10.2 · unitree_sdk2 · RealSense · Go2 sport(고수준) 모드
