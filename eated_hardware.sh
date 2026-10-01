#!/usr/bin/env bash
# Restore ROS library paths after sg switches to the serial-device group.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/fcsl/eated_ws/install/setup.bash
set -u
export LDS_MODEL=LDS-02

exec ros2 launch turtlebot3_manipulation_bringup hardware.launch.py \
    use_fake_hardware:=false start_rviz:=false "$@"
