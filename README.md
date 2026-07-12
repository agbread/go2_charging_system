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
| [`aruco_go2_docking`](aruco_go2_docking/README.md) | Python | 도킹 "두뇌": ArUco 마커 검출 + 도킹 상태머신(접근→정렬→앉기→충전 확인→재시도) + 시뮬용 mock 충전 노드 |
| [`go2_sport_bridge`](go2_sport_bridge/README.md) | C++ | 실제 로봇 어댑터: `/cmd_vel`·`/joy` ↔ SportClient 번역, `rt/lowstate` → `/joint_states`·`/charging_state`. RL 정책 불필요 |

## 빠른 실행

### 시뮬레이션 (Gazebo)

터미널 3개를 열고 순서대로 실행합니다:

```bash
# 터미널 1: Gazebo 환경 + 로봇
ros2 launch aruco_go2_docking aruco_docking_sim.launch.py rname:=go2
# 터미널 2: RL 로코모션 (rl_sar — 아래 참고)
ros2 run rl_sar rl_sim
# 터미널 3: 도킹 노드 + mock 충전
ros2 launch aruco_go2_docking aruco_docking.launch.py
# 충전 성공 시뮬레이션 (아무 터미널에서나)
ros2 param set /mock_charging_node charging_success true
```

> `rl_sar`(RL 보행 컨트롤러)와 `go2_description`은 이 저장소에 포함되어 있지 않습니다.
> [RCILab/RCI_quadruped_robot_navigation](https://github.com/RCILab/RCI_quadruped_robot_navigation)
> 저장소를 같은 워크스페이스(`~/ros2_ws/src`)에 받아 함께 빌드한 뒤 실행하면 됩니다.

### 실제 로봇 (Go2 내장 sport-mode 제어기)

```bash
# 터미널 1: RealSense 카메라
ros2 launch realsense2_camera rs_launch.py
# 터미널 2: 도킹 전체 (어댑터 + detector + controller) — 완료 시 자동 종료
ros2 launch go2_sport_bridge go2_native_docking.launch.py network_interface:=eth0
```

> ※ 실제 로봇은 CycloneDDS 환경설정(`RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`)이 필수입니다.
> 누락 시 어댑터가 `Failed to create domain explicitly`로 즉시 죽습니다 — [ONBOARDING.md](ONBOARDING.md) 4장 참고.

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
| [ONBOARDING.md](ONBOARDING.md) | 새 PC 설치 절차 (의존성 설치, CycloneDDS/DDS 설정, 트러블슈팅) |
| [aruco_go2_docking/README.md](aruco_go2_docking/README.md) | 도킹 패키지 상세 (시뮬 중심) |
| [go2_sport_bridge/README.md](go2_sport_bridge/README.md) | sport-mode 어댑터 상세 (번역 테이블, 보정 포인트) |
| [연동 규격서 PDF](aruco_go2_docking/Unitree%20Go2%20자율%20충전%20상태%20알림%20연동%20규격서.pdf) | `/aruco_state` 연동 규격 원문 |

## 검증 환경

Ubuntu 20.04 (Jetson) · ROS 2 Foxy · CycloneDDS 0.10.2 · unitree_sdk2 · RealSense · Go2 sport(고수준) 모드
