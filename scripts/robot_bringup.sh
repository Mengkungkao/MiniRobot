#!/usr/bin/env bash
set -euo pipefail

# ROS 2 entry point for the mini-robot.service unit written by
# scripts/robot_autostart.sh. Not meant to be run by hand - the manual
# workflow is still `ros2 launch mini_robot robot.launch.py` in a login shell.
#
# Why a wrapper at all: systemd runs a non-interactive shell, and Ubuntu's
# ~/.bashrc returns immediately for those, so the ROS block the bootstrap
# appended there is never reached (README section 21). Everything a ROS node
# needs is therefore sourced explicitly here.
#
# Environment (all set by the unit file, all overridable):
#   ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY WORKSPACE_DIR
#   ARDUINO_PORT LIDAR_PORT PORT_WAIT_SECONDS

ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
ARDUINO_PORT="${ARDUINO_PORT:-/dev/arduino_mini}"
LIDAR_PORT="${LIDAR_PORT:-/dev/coin_d6}"
# udev creates the symlinks a moment after the USB adapters enumerate, which
# on a cold boot can land after this unit starts.
PORT_WAIT_SECONDS="${PORT_WAIT_SECONDS:-30}"

log() { echo "[robot_bringup] $*"; }

for setup in "/opt/ros/${ROS_DISTRO}/setup.bash" "${WORKSPACE_DIR}/install/setup.bash"; do
  if [ ! -f "$setup" ]; then
    log "ERROR: $setup not found. Run scripts/bootstrap_robot_stack.sh pi first." >&2
    exit 1
  fi
  # ROS 2's generated setup.bash references unset vars like COLCON_TRACE.
  set +u
  # shellcheck disable=SC1090
  source "$setup"
  set -u
done

# Stand aside for a hand-started stack. Two processes cannot share the Arduino
# or the LiDAR port, and taking the ports from the one someone is actively
# driving with is worse than this unit not running. Exit 0 so systemd records
# a clean stop instead of restart-looping against a port it will never get.
if pgrep -f 'serial_bridge|coin_d6_lidar' >/dev/null 2>&1; then
  log "the stack is already running outside this unit (hand-started?); standing aside"
  exit 0
fi

deadline=$((SECONDS + PORT_WAIT_SECONDS))
while [ ! -e "$ARDUINO_PORT" ] || [ ! -e "$LIDAR_PORT" ]; do
  if [ "$SECONDS" -ge "$deadline" ]; then
    log "ERROR: waited ${PORT_WAIT_SECONDS}s for $ARDUINO_PORT and $LIDAR_PORT." >&2
    log "Present now: $(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | tr '\n' ' ')" >&2
    log "If the symlinks are missing, run: bash scripts/detect_serial_ports.sh --install" >&2
    exit 1
  fi
  sleep 1
done

log "starting robot.launch.py (arduino=$ARDUINO_PORT lidar=$LIDAR_PORT domain=$ROS_DOMAIN_ID)"
# exec so ros2 launch becomes the unit's main process and receives the SIGINT
# systemd sends on stop, which is what shuts the nodes down cleanly.
exec ros2 launch mini_robot robot.launch.py \
  "arduino_port:=$ARDUINO_PORT" "lidar_port:=$LIDAR_PORT" "$@"
