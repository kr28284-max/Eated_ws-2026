# EATED Service Robot (2026)

ROS 2 Humble 기반의 TurtleBot3 + OpenMANIPULATOR-X 서비스 로봇 프로젝트입니다. Intel RealSense D435i의 RGB-D 영상과 YOLO를 이용해 물체를 찾고, MoveIt 2로 물체를 집은 뒤 사용자가 지정한 위치로 운반합니다.

현재 인식 대상은 다음 네 종류입니다.

- `airpod`
- `coke`
- `tissue`
- `vaseline`

## 주요 기능

- Logitech F710 조이스틱을 이용한 TurtleBot3 수동 주행
- 음성/LLM 결과를 `RobotCommand` 토픽으로 받아 목표 물체 설정
- 필요할 때만 YOLO 추론을 수행하는 `FindObject` Action Server
- RealSense RGB와 aligned depth를 이용한 물체의 3차원 좌표 계산
- `base_link` 좌표계로 변환된 물체 위치를 이용한 Pick 동작
- 물체 파지 후 CARGO 자세로 전환하고 배치 위치까지 수동 주행
- Y 버튼 비상 정지 및 X 버튼 HOME 복구
- Terminator 3×3 레이아웃을 이용한 9개 실행 터미널 준비

## 구성

```text
eated_ws/
├── src/
│   ├── control_pkg/       # 전체 작업 상태 머신과 Pick/Place 제어
│   ├── eated_interfaces/  # RobotCommand 메시지와 FindObject 액션
│   ├── joystick_pkg/      # TurtleBot3 수동 주행 및 버튼 입력
│   └── vision_pkg/        # YOLO + RGB-D 물체 위치 추정
├── best.pt                # airpod/coke/tissue/vaseline YOLO 모델
├── eated                  # 9분할 Terminator 실행 명령
├── eated_pane.sh          # 각 패널의 ROS 명령 준비
├── eated_hardware.sh      # 시리얼 그룹에서 하드웨어 실행
└── eated_terminator.conf  # Terminator 3×3 레이아웃
```

`build/`, `install/`, `log/`는 빌드 결과이므로 Git에 포함하지 않습니다.

## 개발 환경

- Ubuntu 22.04 / ROS 2 Humble
- NVIDIA Jetson Orin
- TurtleBot3 + OpenMANIPULATOR-X
- Intel RealSense D435i
- Logitech F710 (XInput 모드)
- MoveIt 2, `ros2_control`, RealSense ROS 2 wrapper
- Python 3.10, OpenCV, Ultralytics YOLO
- Jetson 환경과 호환되는 CUDA PyTorch/torchvision

추가로 다음 ROS 패키지가 설치되어 있어야 합니다.

- `turtlebot3_manipulation_bringup`
- `turtlebot3_manipulation_moveit_config`
- `realsense2_camera`
- `joy`

## 내려받기 및 빌드

이 프로젝트의 실행 파일에는 현재 로봇 PC의 경로가 반영되어 있으므로 `/home/fcsl/eated_ws`에 클론하는 것을 기준으로 합니다.

```bash
cd /home/fcsl
git clone https://github.com/kr28284-max/Eated_ws-2026.git eated_ws
cd /home/fcsl/eated_ws

sudo apt install python3-colcon-common-extensions python3-rosdep2 python3-pip python3.10-venv python3-cffi \
  ros-humble-turtlebot3-manipulation-bringup ros-humble-turtlebot3-manipulation-moveit-config \
  ros-humble-realsense2-camera ros-humble-moveit ros-humble-joy \
  ros-humble-ld08-driver ros-humble-turtlebot3-manipulation-hardware \
  nvidia-jetpack-runtime cuda-cupti-12-6
sudo usermod -aG dialout "$USER"  # 라이다와 OpenCR 시리얼 포트 접근
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

JetPack 6.2.3 / CUDA 12.6에서는 다음과 같이 프로젝트 전용 Python 패키지를 설치합니다.

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install --no-deps torch==2.8.0 torchvision==0.23.0 \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
.venv/bin/python -m pip install 'numpy<2' 'opencv-python<4.9' ultralytics
```

`eated`의 각 패널은 이 가상환경의 패키지 경로를 자동으로 사용합니다. 패널 밖에서 Vision을 직접 실행할 때는 `export PYTHONPATH="$PWD/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"`를 먼저 실행합니다. 다른 JetPack 버전에서는 호환되는 PyTorch 빌드를 확인해야 합니다.

## 9분할 실행 화면 설치

```bash
sudo apt install terminator
chmod +x /home/fcsl/eated_ws/eated /home/fcsl/eated_ws/eated_pane.sh /home/fcsl/eated_ws/eated_hardware.sh
mkdir -p /home/fcsl/.local/bin
ln -sf /home/fcsl/eated_ws/eated /home/fcsl/.local/bin/eated
```

`~/.local/bin`이 PATH에 없다면 다음 줄을 `~/.bashrc`에 추가합니다.

```bash
export PATH="$HOME/.local/bin:$PATH"
```

그래픽 터미널에서 아래 명령을 실행합니다.

```bash
eated
```

Terminator에 9개 패널이 열리고 각 명령이 입력된 상태로 대기합니다. 명령은 자동 실행되지 않으며, 로봇 상태를 확인하면서 각 패널에서 Enter를 눌러야 합니다.

| 번호 | 역할 | 실행 시점 |
|---:|---|---|
| 1 | TurtleBot3 + OpenMANIPULATOR 하드웨어 Bringup | 가장 먼저 실행하고 controller 활성화 대기 |
| 2 | MoveIt `move_group` | 1번 준비 완료 후 |
| 3 | RealSense D435i | 카메라 연결 확인 후 |
| 4 | Vision Node 2 | 3번 RGB/depth 토픽 확인 후 |
| 5 | `joy_node` | F710을 XInput 모드로 연결한 후 |
| 6 | Joystick Control | 5번 `/joy` 토픽 확인 후 |
| 7 | Control Node | 로봇 주변을 비운 후 실행 |
| 8 | `/voice/robot_command` 구독 메시지 표시 | 다른 PC의 발행 전 실행 가능 |
| 9 | Control 상태 모니터 | 필요할 때 실행 |

> 8번 패널은 토픽을 발행하지 않고 수신한 메시지를 계속 표시합니다. 다른 PC에서 발행할 때 두 PC의 `ROS_DOMAIN_ID`를 동일하게 설정하세요. Control Node는 시작할 때 매니퓰레이터를 HOME 자세로 움직이므로, 실제 명령 발행은 7번 로그에 `Startup complete: joystick drive enabled`가 출력된 뒤에 하세요.

## 기본 동작 순서

1. 패널 1~7을 순서대로 실행합니다.
2. 패널 8에서 토픽 구독을 시작하고 Control Node가 `IDLE`인지 확인합니다.
3. 다른 PC에서 동일한 `ROS_DOMAIN_ID`로 `/voice/robot_command` 명령을 발행합니다.
4. 유효한 fetch 명령을 받으면 바로 인식 자세 이동 → YOLO 탐지 → Pick → CARGO 자세가 순서대로 수행됩니다.
5. 상태가 `DRIVE_TO_PLACE`가 되면 배치 위치까지 수동 주행합니다.
6. D435i 카메라에 손바닥을 연속 2초 펼쳐 보이면 로봇이 정지하고, 현재 CARGO 자세에서 그리퍼를 연 뒤 HOME으로 복귀합니다.

작업 중 일반 오류가 발생하면 Control Node가 팔을 HOME으로 복귀시킵니다. HOME 복귀도 실패하면 `ERROR` 상태에 머무르며 X 버튼으로 재시도할 수 있습니다. Y 버튼 또는 음성 `stop`은 비상 정지로 처리하며 자동 HOME 복귀를 시작하지 않습니다.

F710 XInput 버튼 역할은 다음과 같습니다.

| 버튼 | 기능 |
|---|---|
| A | `IDLE` 상태에서 마지막으로 요청한 물체 인식/Pick 다시 시작 |
| B | 사용하지 않음 |
| X | `ERROR` 상태에서 매니퓰레이터 HOME 복구 |
| Y | 모든 진행 동작 비상 정지 및 조이스틱 주행 권한 복구 |

## 명령 토픽 직접 보내기

Control Node가 `IDLE`인 상태에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/fcsl/eated_ws/install/setup.bash

ros2 topic pub --once /voice/robot_command \
  eated_interfaces/msg/RobotCommand \
  "{action: fetch, target_class: airpod, transcript: '에어팟 가져다줘'}"
```

`target_class`는 `airpod`, `coke`, `tissue`, `vaseline` 중 하나여야 합니다.

노트북의 `local-voice-robot-command` 저장소 `ros2-command-integration` 브랜치는
같은 `/voice/robot_command` 토픽에 `eated_interfaces/msg/RobotCommand`를
발행합니다. `fetch`는 목표 물체를 설정하고, `stop`(빈 `target_class`)은 진행 중인
동작을 취소하고 로봇을 `ERROR` 상태로 전환합니다. Control Node가 `IDLE`일 때
`fetch`를 보내면 `/control/command_status`에서 수락 여부를 확인할 수 있습니다.

## Vision 단독 확인

RealSense와 Vision Node를 실행한 후 다음 액션으로 물체 좌표를 확인할 수 있습니다.

```bash
ros2 action send_goal /find_object \
  eated_interfaces/action/FindObject \
  "{target_class: coke}" --feedback
```

성공하면 결과의 `x`, `y`, `z`는 `frame_id: base_link` 기준의 미터 단위 좌표입니다. 첫 추론에서는 CUDA backend 초기화 때문에 시간이 더 걸릴 수 있으며, 초기화가 끝난 뒤 인식 timeout이 시작됩니다.

## 주요 설정

- Control/Pick 설정: `src/control_pkg/config/control.yaml`
- Vision 설정: `src/vision_pkg/config/vision2.yaml`
- 카메라 장착 TF: `src/vision_pkg/launch/vision2.launch.py`
- 조이스틱 주행 속도: `src/joystick_pkg/joystick_pkg/total_control.py`

현재 Pick 설정은 실제 로봇 기준의 초기값입니다.

- PREGRASP: 검출 위치 위 5 cm
- RETREAT: 파지 직후 실제 툴 위치에서 `base_link` Z 방향으로 최대 15 cm의 직선 경로를 계산하고, MoveIt이 만든 구간만 상승한 뒤 CARGO로 이동
- Pick 목표점: 검출 위치에서 `base_link` +X 방향으로 2 cm 보정
- MoveIt 속도/가속도 배율: 0.10
- RETREAT 직선 경로의 보간 간격: 5 mm (`retreat_cartesian_step_m`)
- 그리퍼 파지 판정: 닫힘 중 stall이 발생하고 관절 위치가 임계값 이상일 때 성공

카메라 장착 위치나 그리퍼 중심이 바뀌면 `pick_calibration_xyz_m`, 카메라 mount TF, 접근/상승 offset을 반드시 다시 실측 보정해야 합니다.

## 손바닥으로 물체 놓기

Vision Node 2와 함께 실행되는 Google MediaPipe Hands 노드가 D435i RGB 영상을
읽습니다. `DRIVE_TO_PLACE` 상태에서만 손바닥을 검사하고, 손가락 네 개가 펼쳐진
상태가 연속 2초 유지되면 `/vision/open_palm` 신호로 그리퍼를 엽니다. 손이
사라지거나 카메라 프레임이 끊기면 시간 계산을 다시 시작합니다. 손 인식 상태는
Vision CV 창에 표시되고, 그리퍼를 열 때까지 남은 시간도 표시됩니다.

Jetson의 작업 공간 가상환경에 MediaPipe가 없다면 다음 명령으로 설치합니다.

```bash
cd /home/fcsl/eated_ws
.venv/bin/python -m pip install --no-deps \
  mediapipe==0.10.18 absl-py==2.3.1 flatbuffers==25.2.10 protobuf==4.25.8
```

이 작업 공간에서는 위 패키지를 설치하고 MediaPipe Hands 초기화를 확인했습니다.
Vision Node 2, Control Node, Joystick Control을 재시작해야 새 동작이 적용됩니다.

## 안전 주의사항

- 실제 로봇을 움직이기 전에 주변 사람과 장애물을 치웁니다.
- 처음 시험할 때는 낮은 속도를 유지하고 언제든 Y 버튼을 누를 수 있게 합니다.
- Y 비상 정지 후에는 X 버튼으로 HOME 복구가 끝나야 정상 작업을 다시 시작할 수 있습니다.
- 비상 정지는 소프트웨어 정지 기능입니다. 하드웨어 전원 차단 장치를 대체하지 않습니다.
