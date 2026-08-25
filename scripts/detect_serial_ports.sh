#!/usr/bin/env bash
set -euo pipefail

# Work out which USB serial adapter is the Arduino and which is the COIN-D6,
# then write a udev rules file that pins each one to a stable symlink.
#
#   bash scripts/detect_serial_ports.sh              probe, write scripts/99-mini-robot.rules
#   bash scripts/detect_serial_ports.sh --install    ... and install it into /etc/udev/rules.d
#   bash scripts/detect_serial_ports.sh --dry-run    probe and report only, write nothing
#   bash scripts/detect_serial_ports.sh --wake-lidar also send START_CMD while probing
#
# Why this exists: both adapters on this robot are CH340s (1a86:7523). They
# share one vendor/product ID, have no serial EEPROM, and report an identical
# ID_SERIAL, so nothing in the USB descriptors can tell them apart. The only
# thing that can is what each one says on the wire, so that is what we check.
#
# The Arduino announces itself in ASCII ('READY,', 'T,', 'INFO,', ...) at
# 115200 baud -- all three sketches now share that rate, so there is only one
# to probe. The COIN-D6 emits 0xAA 0x55 framed binary packets at 230400 and
# free-runs without being asked. Neither test writes anything by default, so
# the LiDAR motor is never commanded to spin.
#
# NOTE: opening a port toggles DTR, which resets an Arduino. Harmless on a
# bench, but do not run this while the robot is driving.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RULES_OUT="$SCRIPT_DIR/99-mini-robot.rules"
INSTALL=0
DRY_RUN=0
WAKE_LIDAR=0

for arg in "$@"; do
  case "$arg" in
    --install)    INSTALL=1 ;;
    --dry-run)    DRY_RUN=1 ;;
    --wake-lidar) WAKE_LIDAR=1 ;;
    -h|--help)    sed -n '3,10p' "${BASH_SOURCE[0]}"; exit 0 ;;
    *)            echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

# A probe needs the port to itself, and resetting the Arduino out from under a
# running bridge is worse than refusing to start.
if pgrep -f 'serial_bridge|coin_d6_lidar' >/dev/null 2>&1; then
  echo "ERROR: the ROS nodes are running and holding the serial ports." >&2
  echo "Stop robot.launch.py first, then re-run this script." >&2
  exit 1
fi

mapfile -t DEVICES < <(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true)
if [ "${#DEVICES[@]}" -eq 0 ]; then
  echo "ERROR: no /dev/ttyUSB* or /dev/ttyACM* devices found. Is anything plugged in?" >&2
  exit 1
fi

echo "Probing ${#DEVICES[@]} serial device(s): ${DEVICES[*]}"
echo "(opening a port resets an Arduino; this takes a few seconds per device)"
echo

PROBE_OUT="$(WAKE_LIDAR="$WAKE_LIDAR" python3 - "${DEVICES[@]}" <<'PY'
import os, sys, time
try:
    import serial
except ImportError:
    print('FATAL pyserial missing: sudo apt install python3-serial', file=sys.stderr)
    sys.exit(2)

WAKE = os.environ.get('WAKE_LIDAR') == '1'
LIDAR_START = bytes([0xAA, 0x55, 0xF0, 0x0F])
LIDAR_STOP  = bytes([0xAA, 0x55, 0xF5, 0x0A])
# Every line the two sketches can emit before anything is asked of them.
ARDUINO_MARKERS = (b'READY,', b'T,', b'INFO,', b'WARN,', b'ERR,', b'ACK,', b'PONG', b'C,')


def lidar_frames(buf):
    """Count 0xAA55 packets whose length field actually chains to the next
    header. A raw byte count alone false-positives on mis-framed ASCII."""
    good = i = 0
    while True:
        i = buf.find(b'\xaa\x55', i)
        if i < 0 or i + 10 > len(buf):
            return good
        n = buf[i + 3]
        if 1 <= n <= 160 and buf[i + 10 + 3 * n:i + 10 + 3 * n + 2] == b'\xaa\x55':
            good += 1
        i += 2


def read_for(dev, baud, window, write=None):
    """Accumulate everything the port says for `window` seconds.

    The buffer is cleared once, immediately after opening - i.e. before the DTR
    reset has let the board boot - so stale pre-reset bytes go but the boot
    banner does not. Flushing after a settle sleep instead would discard the
    single most useful line the Arduino ever prints, and on a board still busy
    with IMU startup calibration nothing else arrives to replace it."""
    try:
        s = serial.Serial(dev, baud, timeout=0.2)
    except Exception as exc:
        return None, str(exc)
    try:
        s.reset_input_buffer()
        if write:
            s.write(write); s.flush()
        buf = bytearray()
        deadline = time.monotonic() + window
        while time.monotonic() < deadline:
            buf.extend(s.read(4096))
        return bytes(buf), None
    finally:
        s.close()


ARDUINO_BAUD = 115200

for dev in sys.argv[1:]:
    role, detail = 'unknown', 'no recognisable traffic'

    # Arduino first: ASCII is unambiguous, so a hit here needs no tie-break.
    # One baud to try, now that every sketch runs at 115200.
    data, err = read_for(dev, ARDUINO_BAUD, 3.5)
    if err:
        role, detail = 'error', err
    elif data and any(m in data for m in ARDUINO_MARKERS):
        line = next((l for l in data.split(b'\n')
                     if any(l.startswith(m) for m in ARDUINO_MARKERS)), b'')
        role = 'arduino'
        detail = f"{ARDUINO_BAUD} baud, said {line.strip().decode('ascii', 'replace')[:58]!r}"

    if role == 'unknown':
        data, err = read_for(dev, 230400, 1.0)
        frames = lidar_frames(data or b'')
        if frames >= 3:
            role, detail = 'lidar', f'230400 baud, {frames} chained 0xAA55 packets'
        elif WAKE:
            data, err = read_for(dev, 230400, 1.5, write=LIDAR_START)
            frames = lidar_frames(data or b'')
            if frames >= 3:
                role = 'lidar'
                detail = f'230400 baud, {frames} packets after START_CMD'
            read_for(dev, 230400, 0.2, write=LIDAR_STOP)   # leave the motor stopped

    print(f'{dev}\t{role}\t{detail}')
PY
)" || { echo "Probe failed." >&2; exit 1; }

ARDUINO_DEV=""; LIDAR_DEV=""
printf '%-16s %-9s %s\n' DEVICE ROLE EVIDENCE
while IFS=$'\t' read -r dev role detail; do
  [ -n "$dev" ] || continue
  printf '%-16s %-9s %s\n' "$dev" "$role" "$detail"
  case "$role" in
    arduino) [ -n "$ARDUINO_DEV" ] && { echo "ERROR: two devices look like the Arduino." >&2; exit 1; }; ARDUINO_DEV="$dev" ;;
    lidar)   [ -n "$LIDAR_DEV" ]   && { echo "ERROR: two devices look like the LiDAR." >&2; exit 1; };   LIDAR_DEV="$dev" ;;
  esac
done <<< "$PROBE_OUT"
echo

# Prefer a real USB serial number when the adapter has one (genuine Arduinos
# with an ATmega16U2 do): it survives being moved to another USB port, which
# ID_PATH does not. CH340s have none, so they fall back to the physical path.
match_for() {
  local dev="$1" serial path
  serial="$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_SERIAL_SHORT=//p')"
  path="$(udevadm info -q property -n "$dev" 2>/dev/null | sed -n 's/^ID_PATH=//p')"
  if [ -n "$serial" ]; then
    echo "ENV{ID_SERIAL_SHORT}==\"$serial\"|serial number $serial (survives moving USB ports)"
  elif [ -n "$path" ]; then
    echo "ENV{ID_PATH}==\"$path\"|physical port $path (valid while the cable stays put)"
  else
    echo "|"
  fi
}

missing=0
for pair in "arduino_mini:$ARDUINO_DEV" "coin_d6:$LIDAR_DEV"; do
  if [ -z "${pair#*:}" ]; then
    echo "ERROR: could not identify the ${pair%%:*} device." >&2
    missing=1
  fi
done
if [ "$missing" -eq 1 ]; then
  echo >&2
  echo "If the LiDAR does not stream until told to, re-run with --wake-lidar" >&2
  echo "(that spins the LiDAR motor). If the Arduino is mid-upload or has no" >&2
  echo "sketch that prints on boot, flash the firmware and try again." >&2
  exit 1
fi

A_MATCH="$(match_for "$ARDUINO_DEV")"; L_MATCH="$(match_for "$LIDAR_DEV")"

if [ "$DRY_RUN" -eq 1 ]; then
  echo "Dry run; not writing $RULES_OUT"
  echo "  arduino_mini <- ${A_MATCH%%|*}"
  echo "  coin_d6      <- ${L_MATCH%%|*}"
  exit 0
fi

cat > "$RULES_OUT" <<RULES
# Stable device names for the mini robot's two USB serial adapters.
#
# GENERATED by scripts/detect_serial_ports.sh on $(date -Is) ($(hostname)).
# Do not hand-edit: re-run the script instead. It identifies each adapter by
# what it transmits, not by its USB descriptors, because both adapters on this
# robot are CH340s that report identical vendor, product and serial strings.
#
# Detected:
#   arduino_mini  $ARDUINO_DEV  matched on ${A_MATCH#*|}
#   coin_d6       $LIDAR_DEV  matched on ${L_MATCH#*|}
#
# Install:
#   bash scripts/detect_serial_ports.sh --install
# or by hand:
#   sudo install -m 0644 -o root -g root scripts/99-mini-robot.rules /etc/udev/rules.d/
#   sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty

SUBSYSTEM=="tty", ${A_MATCH%%|*}, SYMLINK+="arduino_mini", MODE="0660", GROUP="dialout"
SUBSYSTEM=="tty", ${L_MATCH%%|*}, SYMLINK+="coin_d6", MODE="0660", GROUP="dialout"
RULES

echo "Wrote $RULES_OUT"
sed -n '/^SUBSYSTEM/p' "$RULES_OUT" | sed 's/^/  /'

if [ "$INSTALL" -eq 1 ]; then
  echo
  echo "Installing to /etc/udev/rules.d/99-mini-robot.rules (needs sudo)"
  sudo install -m 0644 -o root -g root "$RULES_OUT" /etc/udev/rules.d/99-mini-robot.rules
  sudo udevadm control --reload-rules
  sudo udevadm trigger --subsystem-match=tty
  sleep 1
  echo
  if [ -e /dev/arduino_mini ] && [ -e /dev/coin_d6 ]; then
    ls -l /dev/arduino_mini /dev/coin_d6
    echo "Both symlinks are present."
  else
    echo "WARNING: expected symlinks did not appear. Check 'udevadm test' output:" >&2
    echo "  udevadm test \$(udevadm info -q path -n $ARDUINO_DEV) 2>&1 | tail -20" >&2
    exit 1
  fi
else
  echo
  echo "Not installed. Re-run with --install, or install by hand:"
  echo "  sudo install -m 0644 -o root -g root $RULES_OUT /etc/udev/rules.d/99-mini-robot.rules"
  echo "  sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty"
fi
