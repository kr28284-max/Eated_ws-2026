#!/usr/bin/env bash
# Run one EATED Terminator pane with a command waiting for Enter.
#
# ROS processes are intentionally not started automatically. This script only
# prepares the environment and inserts the assigned command into a Readline
# prompt so the operator can start panes in a safe order.

readonly PANE_NUMBER="${1:-}"
readonly WORKSPACE="/home/fcsl/eated_ws"

if [[ ! "$PANE_NUMBER" =~ ^[1-9]$ ]]; then
    echo "Usage: $0 <pane-number 1-9>" >&2
    exit 2
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
cd "$WORKSPACE" || exit 1

case "$PANE_NUMBER" in
    1)
        title="1. HARDWARE"
        hint="먼저 실행하고 모든 controller가 active가 될 때까지 기다리세요."
        command="export LDS_MODEL=LDS-02 && ros2 launch turtlebot3_manipulation_bringup hardware.launch.py use_fake_hardware:=false start_rviz:=false"
        ;;
    2)
        title="2. MOVEIT"
        hint="1번 하드웨어 Bringup이 준비된 다음 실행하세요."
        command="ros2 launch turtlebot3_manipulation_moveit_config move_group.launch.py use_sim:=false"
        ;;
    3)
        title="3. REALSENSE"
        hint="D435i RGB와 aligned depth 토픽을 시작합니다."
        command="ros2 launch realsense2_camera rs_launch.py camera_namespace:=realsense camera_name:=d435i enable_color:=true enable_depth:=true align_depth.enable:=true pointcloud.enable:=false"
        ;;
    4)
        title="4. VISION"
        hint="3번 RealSense 영상이 나온 다음 실행하세요."
        command="ros2 launch vision_pkg vision2.launch.py model_path:=/home/fcsl/eated_ws/best.pt"
        ;;
    5)
        title="5. JOY DRIVER"
        hint="Logitech F710을 XInput 모드로 두고 실행하세요."
        command="ros2 run joy joy_node --ros-args -p device_id:=0 -p deadzone:=0.05 -p autorepeat_rate:=20.0"
        ;;
    6)
        title="6. JOYSTICK CONTROL"
        hint="5번 /joy 토픽이 나온 다음 실행하세요."
        command="ros2 run joystick_pkg total_control"
        ;;
    7)
        title="7. CONTROL NODE"
        hint="주의: Enter를 누르면 매니퓰레이터가 HOME으로 움직입니다."
        command="ros2 launch control_pkg control.launch.py"
        ;;
    8)
        title="8. AIRPOD COMMAND"
        hint="7번 Control Node가 IDLE이 된 다음 Enter를 누르세요."
        command="ros2 topic pub --once /voice/robot_command eated_interfaces/msg/RobotCommand \"{action: fetch, target_class: airpod, transcript: '에어팟 가져다줘'}\""
        ;;
    9)
        title="9. STATE MONITOR"
        hint="Control Node 상태 변화를 계속 표시합니다."
        command="ros2 topic echo /control/state"
        ;;
esac

printf '\033]0;%s\007' "$title"
printf '\n\033[1;36m[%s]\033[0m\n' "$title"
printf '%s\n' "$hint"
printf '\033[1;33m명령은 아직 실행되지 않았습니다. 준비되면 Enter를 누르세요.\033[0m\n\n'

if [[ "${EATED_PANE_DRY_RUN:-0}" == "1" ]]; then
    printf '%s\n' "$command"
    exit 0
fi

if ! IFS= read -er -i "$command" -p "Enter 실행 > " selected_command; then
    exec bash --noprofile --norc -i
fi

if [[ -n "${selected_command//[[:space:]]/}" ]]; then
    history -s "$selected_command"
    eval "$selected_command"
fi

printf '\n[%s] 명령이 종료되었습니다. 일반 셸로 전환합니다.\n' "$title"
exec bash -i
