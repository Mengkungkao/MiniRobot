#!/usr/bin/env bash
set +u

# Stop everything this project starts, on the workstation and on the robot, and
# put the base back to a known state.
#
#   bash scripts/reset_robot_stack.sh --dry-run     list what would be stopped
#   bash scripts/reset_robot_stack.sh               stop it all
#   bash scripts/reset_robot_stack.sh --local       workstation only, leave the Pi
#
# WHY THIS EXISTS
# A second copy of a stack does not announce itself. Two slam_toolbox instances
# both publish map->odom at 20 Hz, TF consumers see the two alternate, and the
# robot appears to jitter in RViz while sitting perfectly still -- with nothing
# in any log saying so. Two controller_servers fight over the same node name and
# goals abort instantly with phantom recoveries. Both happened on this robot.
# `ros2 launch` in a terminal you closed, or a launch whose RViz segfaulted on
# Ctrl-C, is enough to leave one behind.
#
# So: before starting a session, run this. It is much cheaper than diagnosing
# the symptoms.
#
# The kill patterns match the INSTALLED EXECUTABLE PATHS, not the launch command
# line. Matching a launch command line means also matching this script's own
# process, and pkill happily kills the shell that invoked it -- which is its own
# confusing failure mode.

ROBOT_HOST="${ROBOT_HOST:-minirobot@192.168.0.83}"
DRY_RUN=0
LOCAL_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --local)   LOCAL_ONLY=1 ;;
    --robot=*) ROBOT_HOST="${arg#--robot=}" ;;
    -h|--help) sed -n '3,20p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done

# Desktop-side: Nav2, SLAM and RViz. These are shared ROS packages, so anything
# else on this machine using them will also be stopped -- --dry-run first if
# that matters.
DESKTOP_RE="/opt/ros/[a-z]+/lib/(nav2_[a-z_]+|slam_toolbox|rviz2)/"
# Robot-side: this project's own nodes, plus robot_state_publisher.
ROBOT_RE="(mini_robot/(serial_bridge|coin_d6_lidar|cmd_mux)|robot_state_publisher)"

list_local() { ps -eo pid,cmd | grep -E "$DESKTOP_RE" | grep -v grep; }

echo "== workstation =="
FOUND="$(list_local)"
if [ -z "$FOUND" ]; then
  echo "  nothing running"
else
  echo "$FOUND" | sed 's/^/  /' | cut -c1-110
  if [ "$DRY_RUN" -eq 0 ]; then
    echo "$FOUND" | awk '{print $1}' | xargs -r kill 2>/dev/null
    sleep 4
    echo "$(list_local)" | awk '{print $1}' | xargs -r kill -9 2>/dev/null
    sleep 1
    LEFT="$(list_local | wc -l)"
    echo "  -> stopped, $LEFT remaining"
  fi
fi

if [ "$LOCAL_ONLY" -eq 1 ]; then
  echo
  echo "workstation only; the robot was left alone."
  exit 0
fi

echo
echo "== robot ($ROBOT_HOST) =="
if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$ROBOT_HOST" true 2>/dev/null; then
  echo "  unreachable -- skipped"
  exit 0
fi

if [ "$DRY_RUN" -eq 1 ]; then
  ssh "$ROBOT_HOST" "ps -eo pid,cmd | grep -E '$ROBOT_RE' | grep -v grep" 2>/dev/null \
    | sed 's/^/  /' | cut -c1-110
  echo "  (dry run: nothing stopped, odometry not reset)"
  exit 0
fi

# The Arduino is reset last and separately. Killing the bridge leaves the board
# streaming into nothing; its 500 ms watchdog stops the motors on its own, but
# the integrated pose keeps whatever it accumulated, so a fresh session would
# start with a stale odometry origin.
ssh "$ROBOT_HOST" "bash -s" <<'REMOTE' 2>&1 | sed 's/^/  /'
pkill -f "mini_robot/serial_bridge" 2>/dev/null
pkill -f "mini_robot/coin_d6_lidar" 2>/dev/null
pkill -f "mini_robot/cmd_mux" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
sleep 3
pkill -9 -f "mini_robot/serial_bridge" 2>/dev/null
pkill -9 -f "mini_robot/coin_d6_lidar" 2>/dev/null
sleep 1
# grep -c would count the grep's own entry in the ps snapshot, so filter it out
# rather than subtracting one and printing "-1 remaining" when none are left.
LEFT=$(ps -eo cmd | grep -E "mini_robot/(serial_bridge|coin_d6_lidar|cmd_mux)" | grep -vc grep)
echo "nodes stopped, $LEFT remaining"

python3 - <<'PY'
import serial, time
try:
    s = serial.Serial("/dev/arduino_mini", 115200, timeout=0.2)
except Exception as exc:
    raise SystemExit(f"could not open the Arduino: {exc}")
time.sleep(2.5)              # opening the port resets the board
s.reset_input_buffer()
for cmd in (b"STOP\n", b"CLEAR_ESTOP\n", b"RESET_ODOM\n"):
    s.write(cmd); s.flush(); time.sleep(0.3)
time.sleep(0.5)
seen = None
end = time.time() + 2.0
while time.time() < end:
    line = s.readline().decode("ascii", "ignore").strip()
    if line.startswith("T,"):
        f = line.split(",")
        if len(f) == 23:
            seen = f
s.close()
if seen:
    print(f"arduino reset: pose ({seen[2]}, {seen[3]}) mm  yaw {seen[4]} deg  "
          f"pwm {seen[11]}/{seen[12]}  estop={seen[20]}  status={seen[22]}")
else:
    print("arduino: reset sent, no telemetry read back")
PY
REMOTE

echo
echo "Everything stopped. Start ONE stack, in this order:"
echo "  robot :  ros2 launch mini_robot robot.launch.py"
echo "  desktop: ros2 launch mini_robot desktop_slam.launch.py"
echo "Do not start slam_toolbox or Nav2 separately as well -- desktop_slam.launch.py"
echo "already includes both, and a second copy of either is invisible except as"
echo "a robot that jitters in RViz while standing still."
