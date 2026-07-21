# aruco_go2_docking

ArUco 마커 인식 기반 Unitree Go2 자율 충전 패키지 (ROS 2 Humble).

## Overview

```
Camera topic
     │
     ▼
aruco_detector_node ──→ /aruco/marker_pose
                                │
                                ▼
              aruco_docking_controller_node ──→ /cmd_vel  → Go2 locomotion
                                           ──→ /joy       → sit / standup
                                           ──→ /aruco_state (상태 알림)
```

### 상태 알림 토픽 (`/aruco_state`)

| 값 | 발행 시점 |
|----|----------|
| `aruco_arrive` | 충전 패드 위에 엎드림 확인 |
| `aruco_success` | 약 10초간 충전 상태 유지 성공 → 프로그램 자동 종료 |
| `aruco_failed` | 3회 시도 모두 실패 → 프로그램 자동 종료 |

---

## 패키지 구조

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
```

sport 제어기 버전 launch는 `go2_sport_bridge/launch/go2_native_docking_frontcam.launch.py`
(기존 `go2_native_docking.launch.py`와 같은 위치). 캘리브레이션 결과 파일은
저장소 루트의 `go2_front_calib.yaml`.

---

## 1. ArUco 마커 텍스처 생성 (최초 1회)

```bash
cd ~/ros2_ws/src/RCI_quadruped_robot_navigation/aruco_go2_docking
python3 scripts/generate_aruco_marker.py
```

Gazebo에서 마커가 반전되어 감지 안 될 경우:

```bash
python3 scripts/generate_aruco_marker.py --mirror
```

---

## 2. 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select aruco_go2_docking go2_description
source install/setup.bash
```

---

## 3. 실행 — 시뮬레이션

**Terminal 1**: Gazebo 환경 + 로봇 스폰

```bash
ros2 launch aruco_go2_docking aruco_docking_sim.launch.py rname:=go2
```

**Terminal 2**: RL 로코모션 컨트롤러

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rl_sar rl_sim
```

**Terminal 3**: ArUco 도킹 노드 + mock 충전 노드

```bash
ros2 launch aruco_go2_docking aruco_docking.launch.py
```

충전 성공으로 시뮬레이션하려면:

```bash
ros2 param set /mock_charging_node charging_success true
```

---

## 4. 실행 — 실제 로봇

**사전 조건** (별도 터미널에서 먼저 실행):

```bash
# 1. Go2 RL 컨트롤러
ros2 run rl_sar rl_real_go2_ros

# 2. RealSense 카메라 드라이버
ros2 launch realsense2_camera rs_launch.py
```

**ArUco 도킹 실행** (마커가 정면에 보이는 상태에서):

```bash
ros2 launch aruco_go2_docking aruco_docking_real.launch.py
```

---

## 4-1. 실행 — 실제 로봇 (내장 전면 카메라)

D435i 대신 Go2 내장 전면 카메라(H.264 멀티캐스트, HW 디코드)를 쓰는 경로.
기존 D435i 경로와 병행 운용 가능. **최초 1회 캘리브레이션 → 검증 → 도킹** 순서로 진행.

### (0) 카메라 확인

```bash
# 방법 A — 스냅샷 1장 저장 (헤드리스): 방향/화질/구도 확인
python3 ~/ros2_ws/src/aruco_go2_docking/scripts/calibrate_go2_front.py --snapshot ~/ros2_ws/src/front_check.jpg

# 방법 B — 브라우저 실시간 뷰어: 같은 와이파이의 노트북/폰에서 http://<젯슨IP>:8080
python3 ~/ros2_ws/src/aruco_go2_docking/scripts/go2_front_live_view.py

# 방법 C — ROS 토픽으로 확인
source ~/ros2_ws/install/setup.bash
ros2 run aruco_go2_docking go2_front_camera_node
ros2 topic hz /go2_front/image_raw     # 다른 터미널에서. 기대값 ~14Hz (실측 전달률)
```

수신이 안 되면: 로봇 내부망(eth0) 연결 확인. 스트림 자체 검증은
`gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! application/x-rtp, media=video, encoding-name=H264 ! rtph264depay ! h264parse ! nvv4l2decoder ! fakesink`

### (1) 캘리브레이션 (최초 1회, 로봇 교체 시 재실행)

준비물: **100% 배율로 인쇄한** 체커보드 (기본값: Mark Hedley Jones A4, 25mm 칸,
10x7 내부코너). 인쇄 후 자로 4칸=100mm 검증 — 다르면 실측 칸 길이를 `--square`로 입력.
판판한 하드보드에 들뜸 없이 부착. 로봇은 세워두고(가만히), 사람이 보드를 들고 움직인다.

```bash
python3 ~/ros2_ws/src/aruco_go2_docking/scripts/calibrate_go2_front.py                 # 기본 30장
python3 ~/ros2_ws/src/aruco_go2_docking/scripts/calibrate_go2_front.py --num-frames 40 # 40장 (권장)
```

- 진행 리듬: **위치로 이동 → 1초 멈춤 → 터미널 카운터 +1 → 다음 위치로**.
  움직이는 중엔 안 찍힘(모션블러 방지). 수집 간 최소 3초 간격.
- 보드 거리 30~60cm. 중앙 → 가장자리 4방향 → 모서리 → 기울임(±30~40°) → 원근 순으로
  골고루. **광각(HFOV 100°)이라 가장자리/모서리 커버가 왜곡계수 정확도의 핵심.**
- 결과는 `~/ros2_ws/src/go2_front_calib.yaml`에 저장 (기존 파일은 `.bak` 백업).

**양호 판정 기준:**

| 항목 | 양호 | 비고 |
|---|---|---|
| RMS 재투영 오차 | **< 1.0px** | 1.0~2.0 사용 가능(재시도 권장), > 2.0 재수집 필요 |
| fx vs fy | 거의 같음 (1% 이내) | 크게 다르면 수집 품질 의심 |
| cx, cy | 이미지 중심(640, 360) 근처 | |

참고 실측치(이 로봇): RMS 0.689px, fx=800.4, fy=799.4, cx=634.7, cy=359.4.
※ 스펙 근사 K(fx=537)와 실측(fx=800)이 33% 차이 — 이 카메라는 캘리브레이션 필수.
※ RMS가 계속 2px 이상이면: 보드 휨, 인쇄 배율, 조명 불균일, "움직이며 수집"을 의심.

### (2) 캘리브레이션 적용 확인

```bash
# 터미널 1 — 카메라 브릿지
source ~/ros2_ws/install/setup.bash
ros2 run aruco_go2_docking go2_front_camera_node

# 터미널 2 — frontcam detector
source ~/ros2_ws/install/setup.bash
ros2 run aruco_go2_docking aruco_detector_frontcam_node --ros-args \
  --params-file ~/ros2_ws/install/aruco_go2_docking/share/aruco_go2_docking/config/docking_params_frontcam.yaml
```

터미널 2 시작 로그에서 확인:

```
캘리브레이션 로드됨: ~/ros2_ws/src/go2_front_calib.yaml (rms=0.69)   ← 파일 로드 확인
ArUco frontcam detector ready  marker_id=0 size=0.116m ...          ← marker_size 적용 확인
```

"캘리브레이션 파일 없음 — 근사치 동작" WARN이 뜨면 (1)을 먼저 실행할 것.
`--params-file`을 빼먹으면 marker_size가 기본값(0.2m)으로 잡혀 거리가 통째로 틀어짐.

### (3) 거리 오차 검증 (마커 1m 테스트)

1. ArUco 마커(DICT_4X4_50, ID=0 — 충전 패드에 쓰는 그 마커)를 **카메라 렌즈에서
   줄자로 1m** 위치에 정면으로 세운다 (높이는 카메라 높이와 비슷하게).
2. (2)의 두 노드를 띄우고 터미널 2의 로그를 본다:

```
Detected ID=0  dist=1.0XXm  lateral=...
```

3. **dist가 실측 거리의 ±2~3% 이내면 통과.** 오차가 그보다 크면 대부분 마커 인쇄
   크기 문제 — 자로 마커 한 변(검은 테두리 기준)을 재고, 아래로 역산해 yaml의
   `marker_size`를 실측값으로 교체:

```
실제 marker_size = 설정값 × (실측 거리 / dist 표시값)
예) 설정 0.12, 실측 0.98m인데 dist=1.013 → 0.12 × 0.98/1.013 ≈ 0.116
```

4. `marker_size` 변경 후 `colcon build --packages-select aruco_go2_docking` 재빌드
   (yaml은 install 사본을 읽으므로 빌드해야 반영).

### (4) 정지 거리 확정 (target_distance)

로봇을 충전 패드 정위치(앉으면 단자가 닿는 자리)에 수동으로 세우고, (2)의 두 노드에
controller까지 추가로 띄운다 (**어댑터는 띄우지 말 것** — 로봇이 움직이지 않게):

```bash
source ~/ros2_ws/install/setup.bash
ros2 run aruco_go2_docking aruco_docking_controller_node --ros-args \
  --params-file ~/ros2_ws/install/aruco_go2_docking/share/aruco_go2_docking/config/docking_params_frontcam.yaml
```

로그의 `z_m=0.XXXm` 값을 `docking_params_frontcam.yaml`의 `target_distance`로 입력
→ 재빌드. 이 실측이 카메라 오프셋/마커 크기의 잔여 오차를 한 번에 흡수한다.
같은 로그의 `x_m`이 0에서 크게 벗어나면 마커가 패드 중심선에 안 붙은 것.

### (5) 도킹 실행

```bash
# sport 제어기 버전 (어댑터 포함 4개 노드 일괄 실행, 종료 연동 포함)
ros2 launch go2_sport_bridge go2_native_docking_frontcam.launch.py

# RL 제어기 버전 (rl_sar는 기존처럼 별도 실행)
ros2 launch aruco_go2_docking aruco_docking_frontcam.launch.py
```

로봇은 마커 앞 1.5~2m, sport 모드 기립 상태에서 시작. 접근 중 지그재그/움찔거림이
보이면 yaml의 `pose_filter_alpha: 0.4`, `pose_outlier_dist: 0.15` 활성화 검토.

---

## 5. 토픽 확인

```bash
# 상태 알림 (aruco_arrive / aruco_success / aruco_failed)
ros2 topic echo /aruco_state

# ArUco 마커 pose (마커 감지 중일 때 발행)
ros2 topic echo /aruco/marker_pose

# 속도 명령
ros2 topic echo /cmd_vel

# 디버그 이미지
rqt_image_view /aruco/debug_image
```

---

## 6. 파라미터 (config/docking_params_real.yaml)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `image_topic` | `/camera/camera/color/image_raw` | 사용할 카메라 토픽 |
| `camera_info_topic` | `/camera/camera/color/camera_info` | 카메라 info 토픽 |
| `marker_id` | `0` | 감지할 ArUco 마커 ID |
| `marker_size` | `0.173` | 마커 실물 한 변 길이 [m] |
| `target_distance` | `0.40` | 마커까지 목표 거리 [m] |
| `max_linear_x` | `0.10` | 최대 전진 속도 [m/s] |
| `max_angular_z` | `0.30` | 최대 회전 속도 [rad/s] |
| `sit_confirm_timeout_sec` | `20.0` | 앉기 확인 최대 대기 시간 [s] |
| `charge_check_delay_sec` | `3.0` | 앉기 확인 후 충전 체크 전 대기 [s] |
| `charge_wait_timeout_sec` | `10.0` | 충전 상태 확인 대기 시간 [s] |
| `max_retries` | `2` | 재시도 횟수 (총 3회 시도) |
| `backup_speed` | `0.10` | 재시도 시 후진 속도 [m/s] |
| `backup_max_sec` | `15.0` | 최대 후진 시간 [s] |
| `enable_charging_check` | `true` | false 시 충전 확인 생략 |

---

## 7. Acceptance checklist

- [ ] Gazebo 정상 실행
- [ ] `rqt_image_view /aruco/debug_image`에서 마커 인식 확인
- [ ] `/aruco/marker_pose` 발행 확인
- [ ] 로봇이 마커를 향해 접근
- [ ] 목표 거리 도달 후 앉기 동작
- [ ] `/aruco_state: aruco_arrive` 발행 확인
- [ ] `/aruco_state: aruco_success` 발행 후 프로그램 자동 종료
- [ ] 충전 실패 시 일어나서 재시도 (최대 3회)
- [ ] 3회 모두 실패 시 `/aruco_state: aruco_failed` 발행 후 자동 종료

---

## Troubleshooting

**마커 미감지** — Gazebo에서 텍스처 반전 확인 후 재생성:
```bash
python3 scripts/generate_aruco_marker.py --mirror
```

**로봇이 움직이지 않음** — RL 로코모션 실행 여부 확인:
```bash
ros2 topic echo /cmd_vel
ros2 run rl_sar rl_sim
```

**충전 확인 항상 실패 (시뮬)** — mock 노드 파라미터 확인:
```bash
ros2 param set /mock_charging_node charging_success true
```
