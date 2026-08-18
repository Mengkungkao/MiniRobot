#!/usr/bin/env bash
set -euo pipefail

# Checks that the mini robot stack is wired up correctly. Run on either
# machine; parts of the stack that are not currently launched are reported
# as ".." rather than failures.

WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
  # ROS 2's generated setup.bash references unset vars like COLCON_TRACE,
  # which trips `set -u`; relax it just for the sourcing.
  set +u
  # shellcheck disable=SC1091
  source "$WORKSPACE_DIR/install/setup.bash"
  set -u
fi

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

echo "== ROS 2 environment =="
printenv | grep -E '^(ROS|AMENT|COLCON)' | sort || true

echo
echo "== Package discovery =="
for pkg in mini_robot mini_rviz_plugins; do
  if ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "OK   $pkg found: $(ros2 pkg prefix "$pkg")"
  elif [ "$pkg" = "mini_rviz_plugins" ]; then
    echo "..   $pkg not found (expected on the Pi: it needs rviz_common, desktop only)"
  else
    echo "FAIL $pkg not found by ros2 pkg prefix. Did you colcon build and source install/setup.bash?"
  fi
done

echo
echo "== Python syntax check =="
if [ -d "$WORKSPACE_DIR/src/mini_robot" ]; then
  python3 -m compileall -q "$WORKSPACE_DIR/src/mini_robot" && echo "OK   no Python syntax errors in mini_robot"
else
  echo "SKIP $WORKSPACE_DIR/src/mini_robot not found"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo
  echo "ros2 CLI not on PATH; cannot check the live graph. Install/source ROS 2 first."
  exit 0
fi

NODE_LIST="$(ros2 node list 2>/dev/null || true)"
TOPIC_LIST="$(ros2 topic list 2>/dev/null || true)"
SERVICE_LIST="$(ros2 service list 2>/dev/null || true)"

if [ -z "$NODE_LIST" ]; then
  echo
  echo "== Live ROS graph =="
  echo "No nodes visible yet. Start the stack first, e.g. on the robot:"
  echo "  ros2 launch mini_robot robot.launch.py"
  echo "and/or on the workstation:"
  echo "  ros2 launch mini_robot desktop_slam.launch.py"
  exit 0
fi

check() {
  local kind="$1" name="$2" list="$3"
  if grep -qxF "$name" <<<"$list"; then
    echo "OK   $kind $name"
  else
    echo "..   $kind $name not present (fine if that part of the stack isn't launched right now)"
  fi
}

echo
echo "== Expected robot-side nodes (from robot.launch.py) =="
check node /mini_serial_bridge "$NODE_LIST"
check node /coin_d6_lidar "$NODE_LIST"
check node /mini_cmd_mux "$NODE_LIST"
check node /robot_state_publisher "$NODE_LIST"

echo
echo "== Expected robot-side topics =="
check topic /odom "$TOPIC_LIST"
check topic /imu/data "$TOPIC_LIST"
check topic /joint_states "$TOPIC_LIST"
check topic /scan "$TOPIC_LIST"
check topic /diagnostics "$TOPIC_LIST"
check topic /cmd_vel "$TOPIC_LIST"
check topic /cmd_vel_out "$TOPIC_LIST"
check topic /cmd_vel_teleop "$TOPIC_LIST"
check topic /cmd_vel_manual "$TOPIC_LIST"
check topic /tf "$TOPIC_LIST"

echo
echo "== Expected robot-side services (mini_serial_bridge) =="
check service /base/emergency_stop "$SERVICE_LIST"
check service /base/clear_emergency_stop "$SERVICE_LIST"
check service /base/reset_odometry "$SERVICE_LIST"
check service /base/zero_yaw "$SERVICE_LIST"
check service /base/calibrate_imu "$SERVICE_LIST"
check service /base/enable_magnetometer "$SERVICE_LIST"
check service /base/disable_magnetometer "$SERVICE_LIST"

echo
echo "== Nav2/SLAM nodes (only present when a desktop_*.launch.py is running) =="
check node /controller_server "$NODE_LIST"
check node /planner_server "$NODE_LIST"
check node /bt_navigator "$NODE_LIST"
check node /slam_toolbox "$NODE_LIST"
check node /amcl "$NODE_LIST"

echo
echo "== Full graph dump =="
echo "-- nodes --"; echo "$NODE_LIST"
echo "-- topics --"; echo "$TOPIC_LIST"
echo "-- services --"; echo "$SERVICE_LIST"
