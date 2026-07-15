# Go2 자율 충전(ArUco 도킹) 실행 가이드

Unitree Go2의 ArUco 마커 기반 자율 충전 시스템을 실행하기 위한 **전달용 통합 가이드**입니다.
이 문서 하나로 ① 실행 파일 목록·순서, ② 각 파일의 역할, ③ 환경/의존성 설정, ④ 파라미터/옵션을 모두 확인할 수 있습니다.

지원하는 실행 방식은 두 가지입니다:

| 방식 | 보행 담당 | 충전 판정 |
|---|---|---|
| **A. 시뮬레이션** | RL 정책 (`rl_sar rl_sim`) | mock 노드(수동 토글) |
| **B. 실제 로봇 (내장 제어기)** | Go2 내장 sport-mode 제어기 | BMS(Battery Management System) 실측 (`rt/lowstate`) |

---

## 1. 실행 파일 목록과 실행 순서

### A. 시뮬레이션 (Gazebo + RL 보행)

**0) 최초 1회 — ArUco 마커 텍스처 생성**

```bash
cd ~/ros2_ws/src/go2_charging_system/aruco_go2_docking
python3 scripts/generate_aruco_marker.py
# Gazebo에서 마커가 좌우 반전되어 인식이 안 되면:
python3 scripts/generate_aruco_marker.py --mirror
```

**1) 빌드 (최초 1회 / 코드 변경 시)**

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select aruco_go2_docking go2_description
source install/setup.bash
```

**시뮬레이션 실행**
**2) Gazebo 환경 + 로봇 스폰**

```bash
ros2 launch aruco_go2_docking aruco_docking_sim.launch.py rname:=go2
```

**3) RL 로코모션 컨트롤러** (시뮬에는 sport 제어기가 없으므로 RL이 보행 담당)

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rl_sar rl_sim
```

> `rl_sar`는 이 저장소가 아니라
> [RCILab/RCI_quadruped_robot_navigation](https://github.com/RCILab/RCI_quadruped_robot_navigation)
> 저장소에 있습니다. 해당 저장소를 같은 워크스페이스에 받아 빌드한 뒤 위 명령을 실행하면 됩니다
> (로봇 모델 `go2_description`도 같은 저장소에서 제공).

**4) 터미널 3 — 도킹 노드 (detector + controller + mock 충전)**

```bash
ros2 launch aruco_go2_docking aruco_docking.launch.py
```

**5) 충전 성공/실패 시뮬레이션** (기본값은 실패)

```bash
ros2 param set /mock_charging_node charging_success true   # → "charging success"
ros2 param set /mock_charging_node charging_success false  # → "charging failed"
```

### B. 실제 로봇 — Go2 내장(sport-mode) 제어기 사용

> RL 정책 없이, `go2_sport_bridge`가 도킹 컨트롤러의 명령을
> `unitree_sdk2` SportClient 호출로 번역합니다.

**0) 사전 조건 확인**

- 로봇이 **sport(고수준) 모드** — 내장 보행 제어기가 활성 상태여야 합니다.
- DDS 환경변수가 설정돼 있어야 합니다 (아래 3장 참고). 확인:

```bash
echo $RMW_IMPLEMENTATION   # rmw_cyclonedds_cpp 이어야 함
echo $CYCLONEDDS_URI       # file:///home/<user>/cyclonedds_eth0.xml 이어야 함
```

**1) 터미널 1 — RealSense 카메라 드라이버**

```bash
ros2 launch realsense2_camera rs_launch.py
```

토픽이 `/camera/color/image_raw`, `/camera/color/camera_info`로 나오는지 확인합니다
(다르면 `aruco_go2_docking/config/docking_params_real.yaml`의 `image_topic`/`camera_info_topic` 수정).

**2) 터미널 2 — 도킹 전체 실행** (마커가 정면에 보이는 상태에서)

```bash
ros2 launch go2_sport_bridge go2_native_docking.launch.py network_interface:=eth0
```

이 launch 하나가 `sport_mode_adapter_node` + `aruco_detector_node` +
`aruco_docking_controller_node` 3개를 모두 띄웁니다.

**정상 기동 로그:**

```
[sport_mode_adapter_node-1] ... BalanceStand() on start — robot ready to walk.
[sport_mode_adapter_node-1] ... Sport-mode adapter ready on 'eth0'. ...
[aruco_docking_controller_node-3] ... Docking controller ready ...
[aruco_docking_controller_node-3] ... Waiting for marker...
```

**성공/실패 확인:** 도킹 컨트롤러가 성공(`aruco_success`) / 최종 실패(`aruco_failed`)


### 실행 중 모니터링 (공통)

```bash
ros2 topic echo /aruco_state          # aruco_arrive / aruco_success / aruco_failed
ros2 topic echo /aruco/marker_pose    # 마커 감지 중일 때 발행
ros2 topic echo /cmd_vel              # 속도 명령
ros2 topic echo /charging_state       # "charging success" / "charging failed"
rqt_image_view /aruco/debug_image     # 마커 검출 오버레이 영상
```

---

## 2. 각 파일의 역할

### 노드 (실행되는 프로그램)

| 파일 | 패키지 | 역할 |
|---|---|---|
| `aruco_go2_docking/aruco_detector_node.py` | aruco_go2_docking (Python) | **마커 인식.** 카메라 이미지 + camera_info 구독 → ArUco(DICT_4X4_50, ID=0) 검출 → `solvePnP`로 3D pose 추정 → `/aruco/marker_pose` 발행. 디버그 오버레이는 `/aruco/debug_image`. OpenCV 신(≥4.7)/구(≤4.6) API 모두 지원. |
| `aruco_go2_docking/aruco_docking_controller_node.py` | aruco_go2_docking (Python) | **도킹 제어 상태머신.** `DOCKING → SETTLING → SITTING → CHECKING → DONE/FAILED`, 실패 시 `RETRY_STANDUP`. 마커 고정좌표계 기준 접근·센터링·헤딩 정렬 → 완전 정지 → 앉기(joy B) → 관절각으로 앉음 확인 → 충전 상태 10초 유지 확인. **재시도 로직**: 충전 실패 시 일어서기(joy A) → 보행 재활성(RB+DPadUp) → 마커가 다시 보일 때까지 후진 → 재접근 (총 3회 시도). 결과를 `/aruco_state`로 발행하고 자동 종료. |
| `aruco_go2_docking/mock_charging_node.py` | aruco_go2_docking (Python) | **시뮬 전용.** `aruco_arrive` 수신 후 `/charging_state`를 모의 발행. `charging_success` 파라미터로 성공/실패 토글. |
| `go2_sport_bridge/src/sport_mode_adapter_node.cpp` | go2_sport_bridge (C++) | **실제 로봇 전용 어댑터.** 도킹 컨트롤러의 제너릭 명령을 sport-mode 호출로 번역: `/cmd_vel`→`SportClient::Move`, joy A→`RecoveryStand`(일어서기), B→`StandDown`(엎드림=충전 자세), X→`StandUp`, RB+DPadUp→`BalanceStand`(보행 재활성). 역방향으로 `rt/lowstate`의 관절 상태→`/joint_states`, BMS(status 6/7 또는 충전 전류)→`/charging_state` 발행. StandDown 후에는 `/cmd_vel` 차단. |

### launch / 설정 / 리소스

| 파일 | 역할 |
|---|---|
| `aruco_go2_docking/launch/aruco_docking_sim.launch.py` | 시뮬: Gazebo(월드+마커) + 로봇 스폰 + robot_state_publisher + joint_state_broadcaster |
| `aruco_go2_docking/launch/aruco_docking.launch.py` | 시뮬: detector + controller + mock_charging_node (파라미터: `docking_params.yaml`) |
| `aruco_go2_docking/launch/aruco_docking_real.launch.py` | 실제 로봇(RL 방식용): detector + controller만 (파라미터: `docking_params_real.yaml`) |
| `go2_sport_bridge/launch/go2_native_docking.launch.py` | **실제 로봇 메인 진입점**: 어댑터 + detector + controller 일괄 실행, 종료 연동, CycloneDDS 라이브러리 경로 픽스 포함 |
| `aruco_go2_docking/config/docking_params.yaml` | 시뮬용 파라미터 (카메라 토픽 `/color/image_raw`, 데드존 보정 OFF) |
| `aruco_go2_docking/config/docking_params_real.yaml` | 실제 로봇용 파라미터 (RealSense 토픽, Go2 데드존 보정, 실측 보정값) |
| `cyclonedds_eth0.xml` | CycloneDDS 설정: 도메인 0을 eth0에 바인딩, 멀티캐스트는 디스커버리만(spdp) |
| `aruco_go2_docking/scripts/generate_aruco_marker.py` | Gazebo 텍스처용 마커 PNG 생성 (`--id`, `--size`, `--mirror`) |
| `aruco_go2_docking/models/aruco_marker_board/`, `worlds/aruco_docking_test.world` | Gazebo 마커 모델(0.24m 보드, 중심 높이 0.4m)과 테스트 월드(마커는 스폰 지점 2.5m 전방) |
| `ONBOARDING.md` | 새 PC 설치 절차 상세 (의존성·DDS 설정·트러블슈팅) |
| `aruco_go2_docking/Unitree Go2 자율 충전 상태 알림 연동 규격서.pdf` | `/aruco_state` 연동 규격 원문 |


### `/aruco_state` 상태 알림 규격 (규격서 요약)

| 값 | 발행 시점 | 시스템 동작 |
|---|---|---|
| `aruco_arrive` | 충전 패드 위에 엎드림 확인(관절각 기준) | 충전 확인 대기 |
| `aruco_success` | 약 10초간 충전 상태 유지 성공 | 서비스 자동 종료 |
| `aruco_failed` | 총 3회 시도(초기 1 + 재시도 2) 모두 실패 | 일어선 뒤 자동 종료 |

---

## 3. 실행 전 필요한 환경/의존성 설정

시뮬레이션과 실제 로봇의 기준 환경이 다릅니다.
도킹 패키지(`aruco_go2_docking`)는 두 환경 모두에서 동작합니다.

| 구분 | OS | ROS 2 |
|---|---|---|
| 시뮬레이션 | Ubuntu 22.04 | **Humble** ([RCILab rl_sar](https://github.com/RCILab/RCI_quadruped_robot_navigation) 기준) |
| 실제 로봇 | Ubuntu 20.04 (Jetson/Tegra) | **Foxy** (검증된 환경) |

### 공통

| 항목 | 내용 |
|---|---|
| OpenCV | `python3-opencv` — `cv2.aruco`가 없으면 `pip install opencv-contrib-python` |
| ROS 패키지 | `cv_bridge`, `image_transport`, geometry/sensor/std_msgs |

### 시뮬레이션 전용 (Ubuntu 22.04 + Humble)

```bash
sudo apt install -y \
  ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
  ros-humble-controller-manager ros-humble-robot-state-publisher \
  ros-humble-demo-nodes-cpp \
  python3-opencv
```

- Gazebo 11 + `gazebo_ros`, `controller_manager`, `robot_state_publisher`, `demo_nodes_cpp`(parameter_blackboard)
- **별도 저장소 필요**: `rl_sar`(RL 컨트롤러 — `rl_sim` 실행 파일), `go2_description`(로봇 xacro)
  — 이 저장소에는 포함되어 있지 않습니다.
  [RCILab/RCI_quadruped_robot_navigation](https://github.com/RCILab/RCI_quadruped_robot_navigation)
  저장소를 같은 워크스페이스(`~/ros2_ws/src`)에 clone하여 함께 빌드하세요
  (LibTorch 등 rl_sar 자체 의존성은 해당 저장소 README 참고).

### 실제 로봇 전용 (Ubuntu 20.04 + Foxy)

```bash
sudo apt install -y \
  ros-foxy-rclcpp ros-foxy-rclpy \
  ros-foxy-geometry-msgs ros-foxy-sensor-msgs ros-foxy-std-msgs \
  ros-foxy-cv-bridge ros-foxy-image-transport \
  ros-foxy-realsense2-camera \
  python3-opencv
```

**1) unitree_sdk2** (`/usr/local` 또는 `/opt/unitree_robotics`에 설치):

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
cd ~/unitree_sdk2 && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

**2) CycloneDDS 0.10.2** (Foxy는 직접 빌드 필요 — Humble 이상은 `ros-<distro>-rmw-cyclonedds-cpp`만으로 충분):

```bash
sudo apt install -y ros-foxy-rmw-cyclonedds-cpp
mkdir -p ~/cyclonedds_ws/src && cd ~/cyclonedds_ws/src
git clone -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git
cd ~/cyclonedds_ws && colcon build
```

**3) DDS 환경설정 중요 — 빠지면 어댑터가 기동 즉시 죽습니다**

```bash
cp ~/ros2_ws/src/go2_charging_system/cyclonedds_eth0.xml ~/cyclonedds_eth0.xml
```

`~/.bashrc` 끝에 추가 (Foxy를 source한 *뒤*):

```bash
source ~/cyclonedds_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds_eth0.xml
```

NIC 이름이 `eth0`가 아니면(`ip a`로 확인) `cyclonedds_eth0.xml`의
`<NetworkInterface name="eth0" .../>`를 실제 이름으로 바꿉니다.

**4) 빌드**

```bash
cd ~/ros2_ws
colcon build --packages-select aruco_go2_docking go2_sport_bridge
source install/setup.bash
```

### 주요 트러블슈팅 3가지

| 증상 | 원인 / 해결 |
|---|---|
| `DdsException ... Failed to create domain explicitly` 후 즉사 | DDS 환경변수(3번) 누락 또는 xml 경로 오류. `echo $CYCLONEDDS_URI` 확인. 이 시스템은 SDK에 인터페이스를 넘기지 않고(`ChannelFactory::Init(0)`) ROS가 CYCLONEDDS_URI로 이미 열어둔 도메인 0에 합류하는 구조라, env 설정이 필수입니다. |
| 어댑터가 SIGSEGV(exit -11)로 즉사 | 시스템에 CycloneDDS 코어(libddsc)가 2벌 설치된 경우 — `/usr/local/lib`의 빌드는 SDK와 비호환. `go2_native_docking.launch.py`가 `~/cyclonedds_ws/install/cyclonedds/lib`를 `LD_LIBRARY_PATH` 앞에 자동 삽입해 회피하므로, 반드시 launch로 실행하거나 같은 경로 우선순위를 유지할 것. |
| 노드는 뜨는데 `/joint_states`·`/charging_state`가 안 옴 | xml의 NIC명이 실제 인터페이스와 불일치, 또는 로봇과 다른 서브넷. `ip a`, `ros2 topic echo /joint_states`로 점검. |

기타 트러블슈팅은 [ONBOARDING.md](ONBOARDING.md) 8장 참고.

---

## 4. 실행 시 파라미터/옵션

### launch 인자

| launch | 인자 | 기본값 | 설명 |
|---|---|---|---|
| `aruco_docking_sim.launch.py` | `rname` | `b2` | 로봇 이름 — **Go2는 `rname:=go2` 필수** |
| | `gui` | `true` | Gazebo GUI 실행 여부 |
| `go2_native_docking.launch.py` | `network_interface` | `eth0` | Go2 연결 NIC. ※ 실제 바인딩은 `CYCLONEDDS_URI`의 xml이 담당하고, 이 인자는 호환성용으로만 남아 있음 |

### 주요 config 파라미터 (`config/docking_params*.yaml`)

시뮬(`docking_params.yaml`)과 실제 로봇(`docking_params_real.yaml`) 값이 다른 항목은 `시뮬 / 실제 로봇`으로 표기.

**detector (`aruco_detector_node`)**

| 파라미터 | 값 (시뮬 / 실제 로봇) | 설명 |
|---|---|---|
| `marker_id` | 0 | 감지할 ArUco 마커 ID (DICT_4X4_50) |
| `marker_size` | 0.173 / **0.12** | 마커 실물 한 변 길이 [m] — **실측값과 다르면 거리가 통째로 틀어짐** |
| `image_topic` | `/color/image_raw` / `/camera/color/image_raw` | 사용할 카메라 토픽 (멀티 카메라 환경에서는 여기서 선택) |
| `camera_info_topic` | `/color/camera_info` / `/camera/color/camera_info` | 카메라 내부 파라미터 토픽 |

**controller (`aruco_docking_controller_node`) — 접근/정렬**

| 파라미터 | 값 (시뮬 / 실제 로봇) | 설명 |
|---|---|---|
| `target_distance` | 0.665 / 0.625 | base_link→마커 정지 거리 [m] (마커 법선 방향 기준) |
| `camera_offset_x` / `camera_offset_y` | 0.345 / 0.0 | base_link→카메라 장착 오프셋 [m] |
| `camera_pitch_deg` | 0.0 | 카메라 아래 기울기 [deg] (+ = nose-down). 실제 장착각에 맞춰 입력 |
| `max_linear_x` / `max_angular_z` | 0.20·0.30 / 0.25·0.20 | 속도 상한 [m/s]·[rad/s] |
| `min_linear_x` | 0.20 | **Go2 데드존 보정**: sport 제어기가 ~0.15 m/s 미만 명령을 무시하므로 속도 바닥 적용 |
| `min_angular_z` | 0.0 / 0.1 | yaw 데드존 보정 바닥 (시뮬은 데드존이 없어 0) |
| `goal_tol_dist` / `goal_tol_lateral` | 0.03·0.03 / 0.03·0.06 | 도착 판정 허용 오차 [m] |
| `heading_tol` | 0.12 / 0.14 | 헤딩(몸통 정면) 정렬 임계값 — 거리·중심선·헤딩 3조건 모두 만족해야 앉기 진행 |
| `align_yaw_gain` | -1.0 | 헤딩 정렬 yaw 게인. **로봇이 정렬 중 반대로 돌면 부호 반전** |
| `docking_yaw_release_lat` | 0.10 | 중심선 근처에서 yaw/strafe 순차 정렬 전환 거리(지그재그 방지) |
| `pose_filter_alpha` / `pose_outlier_dist` | 1.0 / 0.0 (OFF) | 실제 카메라 노이즈용 EMA 필터·이상치 제거. 지터가 심하면 예: 0.3 / 0.15 |

**controller — 앉기/충전/재시도**

| 파라미터 | 값 | 설명 |
|---|---|---|
| `sit_thigh_target` / `sit_calf_target` | 1.36 / -2.65 [rad] | 엎드림 판정 관절각. **실제 로봇에서 보정 필요** (아래 참고) |
| `stand_calf_target` | -1.50 [rad] | 일어섬 판정 calf 각 |
| `sit_confirm_timeout_sec` | 20.0 | 관절 기반 앉음 확인 최대 대기 (실제 로봇 GetDown ~10초) |
| `charge_check_delay_sec` | 3.0 | 앉음 확인 후 충전 체크 전 안정화 대기 [s] |
| `charge_wait_timeout_sec` | 10.0 | **규격서: 약 10초간 충전 상태 유지** 확인 시간 [s] |
| `max_retries` | 2 | 재시도 횟수 — **규격서의 총 3회 시도**(초기 1 + 재시도 2) |
| `backup_speed` / `backup_max_sec` | 0.20 / 15.0 | 재시도 시 후진 속도·최대 후진 시간 |
| `enable_charging_check` | true | false면 충전 확인 생략(앉으면 바로 성공 처리) — 접근·정렬만 테스트할 때 유용 |

**sport 어댑터 (`go2_sport_mode_adapter`, 실제 로봇 전용)**

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `max_vx` / `max_vy` / `max_vyaw` | 0.6 / 0.4 / 0.8 | SportClient Move 속도 클램프 |
| `charge_current_threshold_ma` | 0 | 이 값 초과 충전 전류면 충전으로 간주 (BMS status 6/7의 보조 판정) |
| `joy_debounce_sec` | 2.0 | 동일 joy 명령 재발 억제 시간 |
| `balance_stand_on_start` | true | 시작 시 BalanceStand 호출(보행 준비) |

### 실제 로봇 적용 시 보정 포인트 

1. **`marker_size`** — 실제 출력한 마커의 한 변을 자로 재서 입력.
2. **`target_distance`** — 충전 패드 위 정지 위치에 맞게 조정 (base_link 기준, 마커 법선 방향).
3. **`sit_*` / `stand_calf_target`** — 로봇을 한 번 엎드리게 한 뒤 `ros2 topic echo /joint_states`로
   실제 관절각을 읽어 맞출 것 (StandDown/RecoveryStand의 실제 도달각이 기본값과 다를 수 있음).
4. **`align_yaw_gain` 부호** — SETTLING 중 heading 오차가 줄지 않고 커지면 부호를 반전.
5. **`camera_pitch_deg`** — 카메라를 아래로 기울여 장착했다면 실제 각도 입력.

### 런타임 옵션

```bash
# (시뮬) 충전 성공/실패 토글
ros2 param set /mock_charging_node charging_success true

# 마커 검출 확인 (오버레이 영상)
rqt_image_view /aruco/debug_image

# 상위 애플리케이션 연동 지점: 상태 알림 구독
ros2 topic echo /aruco_state
```
