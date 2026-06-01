#!/usr/bin/env bash
set -e

#使用./src/PandaRobot/panda_ros2_gazebo/move_panda.sh home/pose1/pose2/status...来使用

CM="/controller_manager"
CTRL="panda_arm_controller"
TOPIC="/panda_arm_controller/joint_trajectory"

JOINTS="[panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7]"

check_controller_manager() {
  if ! ros2 node list | grep -qx "$CM"; then
    echo "[ERROR] 没找到 $CM"
    echo "请先启动 ros2_control / panda 启动文件"
    exit 1
  fi
}

load_controller_if_needed() {
  local out
  out="$(ros2 control list_controllers -c "$CM" 2>/dev/null || true)"

  if echo "$out" | grep -Fq "$CTRL"; then
    echo "[INFO] 控制器 $CTRL 已加载"
  else
    echo "[INFO] 正在加载控制器 $CTRL ..."
    ros2 control load_controller -c "$CM" "$CTRL"
  fi
}


start_controller_if_needed() {
  local out
  out="$(ros2 control list_controllers -c "$CM" 2>/dev/null || true)"

  if echo "$out" | grep -F "$CTRL" | grep -Eq "active|active$"; then
    echo "[INFO] 控制器 $CTRL 已处于 active"
  else
    echo "[INFO] 正在启动控制器 $CTRL ..."
    ros2 control set_controller_state -c "$CM" "$CTRL" start
  fi
}


show_status() {
  echo "========== controller_manager =========="
  ros2 node list | grep controller_manager || true
  echo
  echo "========== controllers =========="
  ros2 control list_controllers -c "$CM" || true
  echo
  echo "========== topics =========="
  ros2 topic list | grep panda_arm_controller || true
}

send_single_point() {
  local positions="$1"
  local sec="$2"

  ros2 topic pub --once "$TOPIC" trajectory_msgs/msg/JointTrajectory "
joint_names: $JOINTS
points:
- positions: $positions
  time_from_start: {sec: $sec, nanosec: 0}
"
}

send_two_points() {
  local p1="$1"
  local t1="$2"
  local p2="$3"
  local t2="$4"

  ros2 topic pub --once "$TOPIC" trajectory_msgs/msg/JointTrajectory "
joint_names: $JOINTS
points:
- positions: $p1
  time_from_start: {sec: $t1, nanosec: 0}
- positions: $p2
  time_from_start: {sec: $t2, nanosec: 0}
"
}

home() {
  echo "[INFO] 移动到 home 姿态 ..."
  send_single_point "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 4
}

pose1() {
  echo "[INFO] 移动到 pose1 ..."
  send_single_point "[0.0, -0.3, 0.0, -1.6, 0.0, 1.3, 0.5]" 3
}

pose2() {
  echo "[INFO] 通过中间点移动到 pose2 ..."
  send_two_points \
    "[0.0, -0.2, 0.0, -1.0, 0.0, 0.8, 0.2]" 2 \
    "[0.1, -0.6, 0.1, -2.0, 0.1, 1.7, 0.9]" 5
}

usage() {
  echo "用法:"
  echo "  ./move_panda.sh home"
  echo "  ./move_panda.sh pose1"
  echo "  ./move_panda.sh pose2"
  echo "  ./move_panda.sh status"
  exit 1
}

main() {
  if [ $# -lt 1 ]; then
    usage
  fi

  check_controller_manager

  case "$1" in
    status)
      show_status
      ;;
    home|pose1|pose2)
      load_controller_if_needed
      start_controller_if_needed
      show_status
      echo
      case "$1" in
        home) home ;;
        pose1) pose1 ;;
        pose2) pose2 ;;
      esac
      ;;
    *)
      usage
      ;;
  esac
}

main "$@"
