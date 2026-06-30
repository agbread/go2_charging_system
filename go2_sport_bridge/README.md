# go2_sport_bridge

Go2 **내장(sport-mode) 제어기**로 ArUco 충전 도킹을 수행하기 위한 어댑터.
**강화학습(RL) 정책을 전혀 쓰지 않습니다.** 시뮬레이션에서 `rl_sar`(RL 제어기)가
하던 일을, 실기에서는 Go2 공장 보행 제어기(`unitree_sdk2` SportClient)가 대신합니다.

도킹 "두뇌"(`aruco_go2_docking`의 detector/controller)는 **그대로 재사용**하고,
이 패키지는 그 두뇌의 제너릭 명령을 sport-mode 호출로 번역하는 **어댑터 한 개**만 추가합니다.

## 동작 (번역 테이블)

| 도킹 컨트롤러 출력 | → | 이 어댑터가 호출 |
|---|---|---|
| `/cmd_vel` (Twist) | → | `SportClient::Move(vx, vy, vyaw)` |
| `/joy` A (buttons[0]) | → | `SportClient::RecoveryStand()` (일어서기) |
| `/joy` B (buttons[1]) | → | `SportClient::StandDown()` (엎드림 = 충전 자세) |
| `/joy` RB+DPadUp (buttons[5] & axes[7]) | → | `SportClient::BalanceStand()` (보행 준비) |

| 로봇 상태 (`rt/lowstate`) | → | 이 어댑터가 발행 |
|---|---|---|
| `motor_state[0..11].q/dq/tau_est` | → | `/joint_states` (앉음/섬 판정용) |
| `bms_state.status / current` | → | `/charging_state` ("charging success"/"charging failed") |

- **이동 게이팅**: `StandDown` 후에는 `/cmd_vel`을 무시합니다(엎드린 채 `Move(0,0,0)`를
  보내면 로봇이 다시 일어서려 하므로). `BalanceStand`(보행 트리거)에서 다시 활성화됩니다.
- **충전 판정**: BMS `status==7(CHG)` 또는 `6(PRECHG)`, 혹은 `current > 임계값`이면 충전 중.
- **조인트 이름**: `*_thigh_joint`/`*_calf_joint`를 포함해야 컨트롤러의 `_is_sitting`/
  `_is_standing`이 매칭합니다 (이미 그렇게 설정됨).

## 사전 준비

1. 로봇이 **sport(고수준) 모드** — 저수준/RL 제어가 아닌 공장 보행 제어기 활성 상태.
2. `unitree_sdk2`가 `/opt/unitree_robotics`에 설치되어 있을 것 (Unitree `install.sh`).
3. **Foxy**: CycloneDDS 0.10.2를 직접 컴파일하고 `unitree_ros2` DDS 설정(NIC 바인딩)을 적용.
   (Humble은 CycloneDDS 컴파일 생략 가능.)
4. RealSense 드라이버 실행: `ros2 launch realsense2_camera rs_launch.py`

## 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select go2_sport_bridge
```

## 실행

전체 도킹(어댑터 + detector + controller) 한 번에:
```bash
ros2 launch go2_sport_bridge go2_native_docking.launch.py network_interface:=eth0
```

어댑터만 단독 실행:
```bash
ros2 run go2_sport_bridge sport_mode_adapter_node --ros-args -p network_interface:=eth0
```

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `network_interface` | `eth0` | Go2에 연결된 NIC (ChannelFactory 초기화) |
| `max_vx` / `max_vy` / `max_vyaw` | 0.6 / 0.4 / 0.8 | `Move` 속도 클램프 |
| `charge_current_threshold_ma` | 0 | 이 값 초과 충전 전류면 충전으로 간주 (status 보조) |
| `joy_debounce_sec` | 2.0 | 동일 joy 명령 재발 억제 시간 |
| `balance_stand_on_start` | true | 시작 시 `BalanceStand()` 호출(보행 준비) |
| `cmd_vel_topic` / `joy_topic` / `joint_states_topic` / `charging_state_topic` | 표준값 | 토픽 리매핑 |

## 실기 적용 시 보정 포인트 ⚠️

`aruco_go2_docking`의 `config/docking_params_real.yaml`에서:
- `sit_thigh_target` / `sit_calf_target` — Go2 `StandDown`(엎드림) 자세의 실제 관절각으로
  재보정. (RL 애니메이션 기준값과 다를 수 있음 — 한 번 엎드리게 한 뒤 `/joint_states`를 읽어 맞출 것)
- `stand_calf_target` — `RecoveryStand` 후 calf 각도로 보정.
- `target_distance` — 카메라↔마커 거리(베이스↔마커에서 카메라 오프셋 차감).
