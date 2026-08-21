#!/usr/bin/env bash
set -uo pipefail

# The MiniRobot*.h/.cpp files are one library kept as byte-identical copies in
# every sketch folder, because the Arduino IDE compiles a sketch from its own
# directory and nothing else. That arrangement has no compiler-enforced link
# between the copies, so they drift silently -- and drift here is not cosmetic:
# the calibration sketch reports mm/s scaled by these constants and the firmware
# consumes a fit scaled by the same ones, so a mismatch bakes a constant error
# into every speed the robot drives. It has already cost this robot one wasted
# calibration run (firmware had the corrected ENCODER_SIGN, the bench sketch did
# not, and a sweep labelled FORWARD turned the wheel opposite ways in the two
# binaries).
#
#   bash scripts/check_shared_sketch_files.sh          report and exit nonzero on drift
#   bash scripts/check_shared_sketch_files.sh --fix    copy the base controller's
#                                                      copy over the others
#
# Run it before flashing anything, and before extracting these files into a real
# Arduino library.

cd "$(dirname "${BASH_SOURCE[0]}")/.."

REFERENCE=arduino_mini_base_controller
FOLDERS=(arduino_mini_base_controller arduino_mini_motor_calibration arduino_mini_imu_calibration)
SHARED=(MiniRobotPins.h MiniRobotGeometry.h MiniRobotMotors.h MiniRobotMotors.cpp
        MiniRobotEncoders.h MiniRobotEncoders.cpp)

FIX=0
[ "${1:-}" = "--fix" ] && FIX=1

problems=0
for file in "${SHARED[@]}"; do
  reference_path="$REFERENCE/$file"
  if [ ! -f "$reference_path" ]; then
    echo "MISSING REFERENCE  $reference_path"
    problems=$((problems + 1))
    continue
  fi
  want=$(md5sum < "$reference_path")
  for folder in "${FOLDERS[@]}"; do
    path="$folder/$file"
    if [ ! -f "$path" ]; then
      if [ "$FIX" = 1 ]; then
        cp "$reference_path" "$path"; echo "ADDED    $path"
      else
        echo "MISSING  $path"; problems=$((problems + 1))
      fi
    elif [ "$(md5sum < "$path")" != "$want" ]; then
      if [ "$FIX" = 1 ]; then
        cp "$reference_path" "$path"; echo "SYNCED   $path"
      else
        echo "DIFFERS  $path  (vs $reference_path)"; problems=$((problems + 1))
      fi
    fi
  done
done

if [ "$problems" -eq 0 ]; then
  echo "OK: all ${#SHARED[@]} shared files are byte-identical across ${#FOLDERS[@]} sketch folders"
  exit 0
fi
echo
echo "$problems problem(s). Re-run with --fix to copy $REFERENCE's copies over the others,"
echo "but check first that $REFERENCE is the one holding the values you want."
exit 1
