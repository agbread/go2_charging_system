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
│   ├── aruco_detector_node.py          # ArUco 마커 감지 → /aruco/marker_pose
│   ├── aruco_docking_controller_node.py # 도킹 FSM + /aruco_state 발행
│   └── mock_charging_node.py           # 시뮬용: /charging_state 모의 발행
├── config/
│   ├── docking_params.yaml             # 시뮬레이션 파라미터
│   └── docking_params_real.yaml        # 실제 로봇 파라미터
├── launch/
│   ├── aruco_docking.launch.py         # 시뮬: 노드만 (Gazebo 별도 실행 후)
│   ├── aruco_docking_sim.launch.py     # 시뮬: Gazebo 환경 구성
│   └── aruco_docking_real.launch.py    # 실제 로봇 전용
├── models/aruco_marker_board/          # Gazebo 마커 모델
├── scripts/generate_aruco_marker.py
└── worlds/aruco_docking_test.world
```

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
