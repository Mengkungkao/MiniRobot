#!/usr/bin/env bash
set -euo pipefail

# The six MiniRobot* files are one library kept as three physical copies,
# because the Arduino IDE compiles a sketch folder and nothing outside it. They
# describe the same robot -- the same pins, the same counts per revolution, the
# same idea of which way is forward -- so a copy that drifts is a bench sketch
# measuring a robot that does not exist.
#
#   bash scripts/sync_shared_sketch_files.sh --check    report drift, change nothing
#   bash scripts/sync_shared_sketch_files.sh            copy the base controller's
#                                                       copies over the others
#
# --check exits non-zero if anything differs, so it works in a pre-flight or a
# hook. The base controller is the source of truth: it is the one that flies.

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_SKETCH="$REPO_ROOT/arduino_mini_base_controller"
TARGET_SKETCHES=(
  "$REPO_ROOT/arduino_mini_motor_calibration"
  "$REPO_ROOT/arduino_mini_imu_calibration"
)
SHARED_FILES=(
  MiniRobotPins.h
  MiniRobotGeometry.h
  MiniRobotMotors.h
  MiniRobotMotors.cpp
  MiniRobotEncoders.h
  MiniRobotEncoders.cpp
)

check_only=0
[ "${1:-}" = "--check" ] && check_only=1

drift=0
copied=0
for target in "${TARGET_SKETCHES[@]}"; do
  for file in "${SHARED_FILES[@]}"; do
    source_path="$SOURCE_SKETCH/$file"
    target_path="$target/$file"
    rel="$(basename "$target")/$file"

    if [ ! -f "$target_path" ]; then
      # The IMU sketch does not use the encoders, but it still gets the files:
      # one copy set is easier to keep honest than a per-sketch subset.
      echo "MISSING  $rel"
      drift=1
      [ "$check_only" -eq 1 ] || { cp "$source_path" "$target_path"; copied=$((copied + 1)); }
      continue
    fi

    if ! diff -q "$source_path" "$target_path" >/dev/null; then
      echo "DIFFERS  $rel"
      drift=1
      if [ "$check_only" -eq 1 ]; then
        diff -u "$source_path" "$target_path" | sed 's/^/    /' | head -40
      else
        cp "$source_path" "$target_path"
        copied=$((copied + 1))
      fi
    fi
  done
done

if [ "$check_only" -eq 1 ]; then
  if [ "$drift" -eq 0 ]; then
    echo "OK: all ${#SHARED_FILES[@]} shared files match across ${#TARGET_SKETCHES[@]} sketches."
    exit 0
  fi
  echo
  echo "Run 'bash scripts/sync_shared_sketch_files.sh' to copy the base"
  echo "controller's versions over the calibration sketches' copies."
  exit 1
fi

if [ "$copied" -eq 0 ]; then
  echo "Nothing to do: all shared files already match."
else
  echo "Synced $copied file(s) from arduino_mini_base_controller."
  echo "Re-upload any calibration sketch you have flashed."
fi
