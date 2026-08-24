#!/usr/bin/env python3
"""Drive one full CAL run on arduino_mini_motor_calibration and log every line.

    python3 scripts/run_motor_calibration.py
    python3 scripts/run_motor_calibration.py --port /dev/arduino_mini -o cal_raw.log

Doing this by hand means holding a serial terminal open for five minutes
without touching the keyboard -- the sketch treats any incoming byte as an
abort -- and then hoping the scrollback kept all 280 samples. This configures
the sweep, sends CAL, and writes every line to a file, which is what
scripts/analyse_motor_calibration.py then turns into a report.

The robot must be ON BLOCKS with both wheels free, and nothing else may hold
the serial port (stop the ROS stack first).
"""
import argparse
import serial
import time

# PWM 4..72 in steps of 2, two passes (up then down) per direction: 35 steps
# x 2 passes x 4 directions = 280 samples in about five minutes. Wide enough to
# see the curve rather than just fit it, dense enough through the 12..30 band
# the robot actually drives in, and starting at 4 so the dead zone is measured
# instead of extrapolated.
SETUP = ["RANGE,4,72", "STEP,2", "DWELL,400,600", "PASSES,2"]

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--port", default="/dev/arduino_mini")
parser.add_argument("-o", "--output", default="cal_raw.log")
args = parser.parse_args()
PORT, LOG = args.port, args.output

log = open(LOG, "w", buffering=1)

def emit(line):
    log.write(line + "\n")
    print(line, flush=True)

s = serial.Serial(PORT, 500000, timeout=0.2)
time.sleep(3.0)
s.reset_input_buffer()

def drain(seconds):
    end = time.time() + seconds
    while time.time() < end:
        raw = s.readline().decode("ascii", "ignore").strip()
        if raw:
            emit(raw)

def send(cmd, settle=0.4):
    emit(">>> " + cmd)
    s.write((cmd + "\n").encode())
    s.flush()
    drain(settle)

emit("### banner")
send("CONFIG", 1.2)
emit("### configuring the sweep")
for cmd in SETUP:
    send(cmd)
send("CONFIG", 1.2)

emit("### CAL -- nothing is written to the board from here until it finishes")
s.write(b"CAL\n")
s.flush()

start = time.time()
last_rx = time.time()
while True:
    raw = s.readline().decode("ascii", "ignore").strip()
    if raw:
        emit(raw)
        last_rx = time.time()
        if raw.startswith("ACK,CAL_DONE") or raw.startswith("ERR,CAL_ABORTED"):
            break
    if time.time() - last_rx > 60:
        emit("### ABORTED: 60 s with no output from the board")
        break
    if time.time() - start > 900:
        emit("### ABORTED: run exceeded 900 s")
        break

emit("### elapsed %.1f s" % (time.time() - start))
# RESULT reprints the fits and the paste block, which is the bit that goes into
# Config.h. Safe now: the sweep is over.
send("RESULT", 2.0)
s.close()
log.close()
print(f"\nraw log written to {LOG}")
print("next: python3 scripts/analyse_motor_calibration.py "
      f"{LOG} -o arduino_mini_motor_calibration/cal_result.md")
