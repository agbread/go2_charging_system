# Go2 Charging System — 새 PC 온보딩(설치) 가이드

Unitree **Go2**의 ArUco 마커 기반 자율 충전(도킹) 시스템을 **새 온보딩 PC**에
설치하고 실행하는 절차입니다. 강화학습(RL) 정책은 쓰지 않고, Go2 내장 sport-mode
보행 제어기(`unitree_sdk2` SportClient)를 사용합니다.

구성 패키지:
- **`go2_sport_bridge`** (C++) — `/cmd_vel`·`/joy` ↔ SportClient 어댑터, `rt/lowstate` → `/joint_states`·`/charging_state`
- **`aruco_go2_docking`** (Python) — 카메라 → ArUco 검출 → 도킹 상태머신

> ⚠️ **가장 흔한 함정**: 이 시스템은 ROS 2가 **CycloneDDS**로 동작하고 환경변수
> `CYCLONEDDS_URI`가 로봇 NIC(eth0)에 바인딩돼 있어야 합니다. 이게 빠지면
> `sport_mode_adapter_node`가 `Failed to create domain explicitly`로 죽거나,
> 로봇 토픽(`rt/lowstate`)을 못 받습니다. **4단계를 반드시 수행하세요.**

---

## 0. 전제 환경

| 항목 | 검증된 버전 |
|---|---|
| OS | Ubuntu 20.04 (Jetson/Tegra 포함) |
| ROS 2 | **Foxy** |
| DDS | **CycloneDDS 0.10.2** (Foxy는 직접 컴파일 필요) |
| Unitree SDK | **unitree_sdk2** (`/usr/local` 또는 `/opt/unitree_robotics`) |
| 카메라 | RealSense (`realsense2_camera`) |
| 로봇 모드 | **sport(고수준) 모드** — 공장 보행 제어기 활성 |

---

## 1. 의존 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-foxy-rclcpp ros-foxy-rclpy \
  ros-foxy-geometry-msgs ros-foxy-sensor-msgs ros-foxy-std-msgs \
  ros-foxy-cv-bridge ros-foxy-image-transport \
  ros-foxy-realsense2-camera \
  python3-opencv
```
- 도킹 검출에는 **OpenCV의 ArUco 모듈**(`cv2.aruco`)이 필요합니다. `python3-opencv`로
  보통 충분하지만, `cv2.aruco`가 없다면 `pip install opencv-contrib-python`로 보완하세요.

## 2. unitree_sdk2 설치

Unitree 공식 SDK를 설치합니다 (헤더/라이브러리가 `/usr/local` 또는
`/opt/unitree_robotics`에 깔립니다).

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git ~/unitree_sdk2
cd ~/unitree_sdk2
mkdir build && cd build
cmake ..          # 기본 설치 경로 /usr/local  (원하면 -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics)
make -j$(nproc)
sudo make install
```
> `go2_sport_bridge/CMakeLists.txt`는 `/opt/unitree_robotics`를 탐색 경로에 추가하고
> `find_package(unitree_sdk2)`로 찾습니다. `/usr/local`에 설치하면 기본 경로라 그대로 잡힙니다.

## 3. CycloneDDS 0.10.2 (Foxy 전용)

Foxy 기본 RMW는 FastDDS라, CycloneDDS를 직접 빌드해 워크스페이스에 둡니다.

```bash
sudo apt install -y ros-foxy-rmw-cyclonedds-cpp   # rmw 어댑터
mkdir -p ~/cyclonedds_ws/src && cd ~/cyclonedds_ws/src
git clone -b 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git
cd ~/cyclonedds_ws && colcon build
```
> Humble 이상에서는 CycloneDDS 직접 컴파일 없이 `ros-<distro>-rmw-cyclonedds-cpp` 설치로 충분합니다.

## 4. DDS 환경설정 ⚠️ (핵심)

저장소에 포함된 **`cyclonedds_eth0.xml`**(도메인 0 / eth0 바인딩)을 홈으로 복사하고
환경변수를 설정합니다.

```bash
# 저장소를 ~/ros2_ws/src 로 받았다고 가정
cp ~/ros2_ws/src/cyclonedds_eth0.xml ~/cyclonedds_eth0.xml
```

`~/.bashrc` 끝에 추가 (Foxy를 source한 *뒤*):
```bash
source ~/cyclonedds_ws/install/setup.bash          # (Foxy에서 직접 빌드한 CycloneDDS)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds_eth0.xml
```
적용: `source ~/.bashrc`

확인:
```bash
echo $RMW_IMPLEMENTATION   # rmw_cyclonedds_cpp
echo $CYCLONEDDS_URI       # file:///home/<user>/cyclonedds_eth0.xml
```

> **NIC 이름이 eth0가 아니면** `cyclonedds_eth0.xml`의
> `<NetworkInterface name="eth0" .../>`를 실제 인터페이스명(`ip a`로 확인)으로 바꾸세요.

## 5. 워크스페이스 빌드

```bash
# 저장소 받기 (워크스페이스 src 로)
git clone https://github.com/agbread/go2_charging_system.git ~/ros2_ws/src

cd ~/ros2_ws
colcon build --packages-select aruco_go2_docking go2_sport_bridge
source install/setup.bash
```

## 6. 실행

1. 카메라 드라이버:
   ```bash
   ros2 launch realsense2_camera rs_launch.py
   ```
   토픽이 `/camera/color/image_raw`, `/camera/color/camera_info`로 나와야 합니다
   (다르면 `aruco_go2_docking/config/docking_params_real.yaml`의 `image_topic`/`camera_info_topic` 수정).

2. 로봇을 **sport 모드**로 두고, 도킹 전체 실행:
   ```bash
   ros2 launch go2_sport_bridge go2_native_docking.launch.py network_interface:=eth0
   ```

정상 기동 로그:
```
[sport_mode_adapter_node-1] ... BalanceStand() on start — robot ready to walk.
[sport_mode_adapter_node-1] ... Sport-mode adapter ready on 'eth0'. ...
[aruco_docking_controller_node-3] ... Docking controller ready ...
[aruco_docking_controller_node-3] ... Waiting for marker...
```

---

## 7. 실기 보정 포인트 ⚠️

`aruco_go2_docking/config/docking_params_real.yaml`:
- `marker_size` — 실제 출력한 ArUco 마커의 한 변 길이[m] (기본 0.173, A4 기준)
- `target_distance` — 카메라↔마커 정지 거리[m] (base_link↔마커 거리에서 카메라 오프셋 차감)
- `sit_*` / `stand_calf_target` — Go2 `StandDown`/`RecoveryStand` 실제 관절각에 맞춰 보정
  (한 번 엎드리게 한 뒤 `/joint_states`를 읽어 맞출 것)

---

## 8. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `terminate ... DdsException ... Failed to create domain explicitly` | 4단계 DDS 환경설정 누락 또는 `CYCLONEDDS_URI` 파일 경로 오류. `echo $CYCLONEDDS_URI` 확인, xml이 홈에 있는지 확인. |
| 노드는 뜨는데 `/joint_states`·`/charging_state`가 안 옴 | `cyclonedds_eth0.xml`의 NIC명이 실제 인터페이스와 불일치, 또는 로봇과 다른 서브넷. `ip a`·`ros2 topic echo /joint_states`로 점검. |
| `find_package(unitree_sdk2) ... not found` (빌드) | 2단계 미수행. `/usr/local` 또는 `/opt/unitree_robotics`에 설치됐는지 확인. |
| `module 'cv2' has no attribute 'aruco'` | `pip install opencv-contrib-python` |
| `Waiting for marker...`에서 멈춤 | 카메라 토픽명 불일치 또는 마커 미검출. `ros2 topic hz /camera/color/image_raw`, 마커 ID/크기 확인. |

---

### 참고: 왜 `network_interface`를 SDK에 안 넘기나
`sport_mode_adapter_node`는 `ChannelFactory::Init(0)`을 **인터페이스 인자 없이** 호출합니다.
ROS 2가 이미 `CYCLONEDDS_URI`로 도메인 0을 eth0에 열어둔 상태라, SDK가 도메인을 **다시
명시 생성**하면 CycloneDDS가 거부(`Failed to create domain explicitly`)하기 때문입니다.
인터페이스 바인딩은 `cyclonedds_eth0.xml`(=env)이 전담합니다. launch의 `network_interface`
인자는 호환성을 위해 남아있지만 SDK로 전달되지 않습니다.
