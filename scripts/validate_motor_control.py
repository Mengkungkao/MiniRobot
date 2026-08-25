#!/usr/bin/env python3
"""Bench validation for the mini base controller's motor control chain.

Speaks the Arduino serial protocol directly (115200 baud, ASCII lines), so it
works whether or not the ROS 2 stack is running -- and it deliberately does NOT
go through /cmd_vel, so a fault in the bridge or the mux cannot be mistaken for
a fault in the firmware.

Why this exists: every speed the robot drives is closed-loop on the encoders,
so the robot's real speed is only as correct as MEASURED_COUNTS_PER_REV. If
that constant is too high the encoders under-report, the PID pushes harder to
compensate, and the robot outruns its command by exactly the ratio -- silently,
with telemetry that looks perfect. Nothing on the robot can detect that on its
own: it needs one ruler measurement from outside. That is test 'scale'.

  bash:  python3 scripts/validate_motor_control.py --port /dev/arduino_mini

Tests, in the order they should be run (each one assumes the earlier ones
passed -- a wrong linear scale makes the track-width result wrong too):

  link      serial link, boot banner, telemetry rate, e-stop/watchdog state
  signs     ON BLOCKS. Both wheels turn the commanded way and measure positive
  ladder    ON BLOCKS. Closed-loop tracking up to MAX_LINEAR_MM_S, PWM headroom,
            and a re-fit of the feed-forward line to compare against Config.h
  watchdog  ON BLOCKS. Motors stop within COMMAND_WATCHDOG_MS of silence
  estop     ON BLOCKS. ESTOP latches, suppresses VEL, and CLEAR_ESTOP releases
  scale     ON THE FLOOR, needs a tape measure. Ground-truth mm/s vs commanded
  track     ON THE FLOOR, needs floor space. In-place turn, IMU as ground truth

  python3 scripts/validate_motor_control.py --tests link,signs,ladder
  python3 scripts/validate_motor_control.py --tests scale --distance-mm 1000
  python3 scripts/validate_motor_control.py --selftest      # no hardware

--selftest runs no hardware at all: it drives a model of the firmware's own
control law with the constants currently in the sketch, and prints what each
failure mode does to the robot's real speed. Use it to read the numbers below
without a robot on the desk.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

REPO_ROOT = Path(__file__).resolve().parent.parent
SKETCH_DIR = REPO_ROOT / "arduino_mini_base_controller"

# 23 with the status word; firmware from before it emitted 22 and is otherwise
# identical on the wire.
TELEMETRY_FIELD_COUNTS = (23, 22)
DEFAULT_PORT = "/dev/arduino_mini"
DEFAULT_BAUD = 115200

# The firmware stops the motors after COMMAND_WATCHDOG_MS without a VEL line,
# so anything that drives has to keep a stream running underneath it.
COMMAND_STREAM_HZ = 20.0

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"


# -----------------------------------------------------------------------------
# What the firmware was compiled with
# -----------------------------------------------------------------------------

@dataclass
class SketchConstants:
    """The constants the running firmware should have been built from.

    Read out of the sketch rather than duplicated here, so the corrected values
    this script prints are relative to what is actually compiled in -- pasting a
    correction computed against a stale copy of the constants is how a scale
    error survives two calibration rounds.
    """
    counts_per_rev: list[float]
    wheel_diameter_mm: float
    track_width_mm: float
    max_linear_mm_s: float
    max_angular_deg_s: float
    max_wheel_mm_s: float
    max_drive_pwm: int
    encoder_sign: list[int]
    motor_direction_sign: list[int]
    wheel_y_sign: list[int]
    pid_kp: float
    pid_ki: float
    pid_kd: float
    max_integral_pwm: float
    ff_breakaway_pwm: list[float]
    speed_window_max_ms: float
    stall_speed_mm_s: float
    ff_slope_fwd: list[float]
    ff_intercept_fwd: list[float]
    ff_slope_rev: list[float]
    ff_intercept_rev: list[float]

    @property
    def mm_per_count(self) -> list[float]:
        return [math.pi * self.wheel_diameter_mm / n for n in self.counts_per_rev]


def _scalar(text: str, name: str, cast=float):
    match = re.search(rf"^const\s+\S+\s+{name}\s*=\s*([-\d.]+)f?\s*;", text, re.M)
    if match is None:
        raise ValueError(f"cannot find {name} in the sketch")
    return cast(float(match.group(1)))


def _array(text: str, name: str, cast=float) -> list:
    match = re.search(rf"^const\s+\S+\s+{name}\s*\[[^\]]*\]\s*=\s*\{{([^}}]*)\}};", text, re.M)
    if match is None:
        raise ValueError(f"cannot find {name} in the sketch")
    # Tolerates the calibration sketch's "0.000f /*NOT_CALIBRATED*/" markers.
    body = re.sub(r"/\*.*?\*/", "", match.group(1))
    return [cast(float(v)) for v in re.findall(r"-?\d+(?:\.\d+)?", body)]


def load_sketch_constants(sketch_dir: Path = SKETCH_DIR) -> SketchConstants:
    config = (sketch_dir / "Config.h").read_text()
    geometry = (sketch_dir / "MiniRobotGeometry.h").read_text()
    return SketchConstants(
        counts_per_rev=_array(geometry, "MEASURED_COUNTS_PER_REV"),
        wheel_diameter_mm=_scalar(geometry, "WHEEL_DIAMETER_MM"),
        track_width_mm=_scalar(config, "TRACK_WIDTH_MM"),
        max_linear_mm_s=_scalar(config, "MAX_LINEAR_MM_S"),
        max_angular_deg_s=_scalar(config, "MAX_ANGULAR_DEG_S"),
        max_wheel_mm_s=_scalar(config, "MAX_WHEEL_MM_S"),
        max_drive_pwm=_scalar(geometry, "MAX_DRIVE_PWM", cast=int),
        encoder_sign=_array(geometry, "ENCODER_SIGN", cast=int),
        motor_direction_sign=_array(geometry, "MOTOR_DIRECTION_SIGN", cast=int),
        wheel_y_sign=_array(geometry, "WHEEL_Y_SIGN", cast=int),
        ff_slope_fwd=_array(config, "FF_SLOPE_MM_S_PER_PWM_FWD"),
        ff_intercept_fwd=_array(config, "FF_INTERCEPT_MM_S_FWD"),
        ff_slope_rev=_array(config, "FF_SLOPE_MM_S_PER_PWM_REV"),
        ff_intercept_rev=_array(config, "FF_INTERCEPT_MM_S_REV"),
        pid_kp=_scalar(config, "WHEEL_PID_KP"),
        pid_ki=_scalar(config, "WHEEL_PID_KI"),
        pid_kd=_scalar(config, "WHEEL_PID_KD"),
        max_integral_pwm=_scalar(config, "MAX_INTEGRAL_PWM"),
        ff_breakaway_pwm=_array(config, "FF_BREAKAWAY_PWM"),
        speed_window_max_ms=_scalar(config, "SPEED_WINDOW_MAX_MS"),
        stall_speed_mm_s=_scalar(config, "STALL_SPEED_MM_S"),
    )


# -----------------------------------------------------------------------------
# Telemetry
# -----------------------------------------------------------------------------

@dataclass
class Sample:
    """One decoded 'T,...' line. Field order matches Telemetry.cpp."""
    t_host: float
    arduino_ms: int
    x_mm: float
    y_mm: float
    yaw_deg: float
    vx_mm_s: float
    wz_deg_s: float
    ticks: list[int]
    wheel_mm_s: list[float]
    wheel_pwm: list[float]
    estop: bool
    watchdog: bool
    status: int

    @classmethod
    def parse(cls, line: str) -> Optional["Sample"]:
        f = line.split(",")
        if len(f) not in TELEMETRY_FIELD_COUNTS:
            return None
        try:
            return cls(
                t_host=time.monotonic(),
                arduino_ms=int(f[1]),
                x_mm=float(f[2]), y_mm=float(f[3]), yaw_deg=float(f[4]),
                vx_mm_s=float(f[5]), wz_deg_s=float(f[6]),
                ticks=[int(v) for v in f[7:9]],
                wheel_mm_s=[float(v) for v in f[9:11]],
                wheel_pwm=[float(v) for v in f[11:13]],
                estop=bool(int(f[20])), watchdog=bool(int(f[21])),
                status=int(f[22]) if len(f) > 22 else 0,
            )
        except ValueError:
            return None


class ArduinoLink:
    """Serial link to the base controller with a background reader thread.

    Opening the port toggles DTR and resets the Uno, so connect() waits for the
    board to boot and re-announce itself rather than trusting the first bytes
    that arrive.
    """

    def __init__(self, port: str, baud: int) -> None:
        import serial  # imported here so --selftest needs no pyserial

        self.port_name = port
        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0.05)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.samples: list[Sample] = []
        self.text_lines: list[str] = []
        self.ready_banner: Optional[str] = None
        # True once the board's CFG reply has been matched against the sketch
        # source; False if it disagrees or the firmware is too old to answer.
        self.firmware_matches_source: Optional[bool] = None
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        buffer = bytearray()
        while not self._stop.is_set():
            try:
                chunk = self._serial.read(4096)
            except Exception:
                break
            if not chunk:
                continue
            buffer.extend(chunk)
            while b"\n" in buffer:
                raw, _, rest = buffer.partition(b"\n")
                buffer[:] = rest
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                if line.startswith("T,"):
                    sample = Sample.parse(line)
                    if sample is not None:
                        with self._lock:
                            self.samples.append(sample)
                else:
                    with self._lock:
                        self.text_lines.append(line)
                    if line.startswith("READY,"):
                        self.ready_banner = line

    def send(self, command: str) -> None:
        self._serial.write((command.strip() + "\n").encode("ascii"))
        self._serial.flush()

    def since(self, t0: float) -> list[Sample]:
        with self._lock:
            return [s for s in self.samples if s.t_host >= t0]

    def text_since(self, t0_index: int) -> list[str]:
        with self._lock:
            return self.text_lines[t0_index:]

    @property
    def text_count(self) -> int:
        with self._lock:
            return len(self.text_lines)

    def latest(self) -> Optional[Sample]:
        with self._lock:
            return self.samples[-1] if self.samples else None

    def wait_for_telemetry(self, timeout: float = 6.0) -> bool:
        deadline = time.monotonic() + timeout
        start = time.monotonic()
        while time.monotonic() < deadline:
            if self.since(start):
                return True
            time.sleep(0.05)
        return False

    def drive(self, linear_mm_s: float, angular_deg_s: float, seconds: float,
              settle: float = 0.0) -> list[Sample]:
        """Stream one velocity for 'seconds' and return the samples taken after
        'settle'. The stream runs at COMMAND_STREAM_HZ throughout, which is what
        keeps the firmware watchdog from cutting the motors mid-test."""
        period = 1.0 / COMMAND_STREAM_HZ
        started = time.monotonic()
        sample_from = started + settle
        next_tx = started
        while True:
            now = time.monotonic()
            if now - started >= seconds:
                break
            if now >= next_tx:
                self.send(f"VEL,{linear_mm_s:.2f},{angular_deg_s:.2f}")
                next_tx += period
            time.sleep(0.005)
        return self.since(sample_from)

    def stop(self) -> None:
        try:
            self.send("STOP")
        except Exception:
            pass

    def close(self) -> None:
        self.stop()
        time.sleep(0.1)
        self._stop.set()
        self._reader.join(timeout=1.0)
        try:
            self._serial.close()
        except Exception:
            pass


# -----------------------------------------------------------------------------
# Results
# -----------------------------------------------------------------------------

@dataclass
class Result:
    name: str
    status: str = SKIP
    detail: str = ""
    notes: list[str] = field(default_factory=list)
    fixes: list[str] = field(default_factory=list)

    def add(self, line: str) -> None:
        self.notes.append(line)

    def fix(self, line: str) -> None:
        self.fixes.append(line)

    def worst(self, status: str, detail: str = "") -> None:
        order = {SKIP: 0, PASS: 1, WARN: 2, FAIL: 3}
        if order[status] >= order[self.status]:
            if order[status] > order[self.status] or not self.detail:
                self.detail = detail or self.detail
            self.status = status


def mean(values) -> float:
    values = list(values)
    return sum(values) / len(values) if values else float("nan")


def ask(prompt: str, assume_yes: bool) -> bool:
    if assume_yes:
        print(f"{prompt} [auto-yes]")
        return True
    try:
        return input(f"{prompt} [y/N] ").strip().lower().startswith("y")
    except EOFError:
        return False


def ask_float(prompt: str) -> Optional[float]:
    try:
        text = input(prompt).strip()
    except EOFError:
        return None
    if not text:
        return None
    try:
        return float(text)
    except ValueError:
        print("  not a number, skipping")
        return None


def ask_wrong_wheels(prompt: str, assume_yes: bool) -> Optional[list[int]]:
    """Ask which wheels physically drove the wrong way.

    Returns the wheel indices that went backwards, an empty list if both were
    correct, or None if nobody actually looked -- which is not the same answer
    and must not be recorded as a pass. Bare Enter means None for the same reason:
    this is the one check telemetry cannot make, so it defaults to "unproven"
    rather than "fine".
    """
    if assume_yes:
        print(f"{prompt} [--yes: not observed]")
        return None
    try:
        answer = input(f"{prompt} [none/1/2/both] ").strip().lower()
    except EOFError:
        return None
    if answer in ("none", "n", "neither", "ok"):
        return []
    if answer in ("1", "m1", "motor1", "motor 1"):
        return [MOTOR_1_INDEX]
    if answer in ("2", "m2", "motor2", "motor 2"):
        return [MOTOR_2_INDEX]
    if answer.startswith("b"):
        return [MOTOR_1_INDEX, MOTOR_2_INDEX]
    print("  no answer recorded -- treating the wheel directions as unverified")
    return None


# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------

# Motor numbering as the firmware uses it: 1 = TB6612 channel A, 2 = channel B.
# Same order as every per-wheel array in the sketch and every per-wheel pair in
# the telemetry line. A motor number names a channel and never a side of the
# robot -- WHEEL_Y_SIGN in MiniRobotGeometry.h is the one place that says which
# side each drives, so this script reports faults by motor number and lets the
# operator look up the wheel.
MOTOR_1_INDEX, MOTOR_2_INDEX = 0, 1
WHEEL = ("motor 1", "motor 2")


def check_firmware_matches_source(link: ArduinoLink, k: SketchConstants, r: Result) -> None:
    """Prove the board is running the sketch this script is reading.

    Everything else here compares the robot against constants loaded from
    Config.h and MiniRobotGeometry.h. If the board was flashed before those
    files were last edited, every corrected value this script prints is
    computed from numbers the robot is not using -- and the sign advice in
    particular inverts: told to "flip MOTOR_DIRECTION_SIGN", an operator whose
    source already contains the fix would flip it back out and restore the
    runaway. So this is a hard stop, not a warning.

    The board answers CFG with its compiled-in constants and also volunteers
    them at boot. Firmware older than that command answers nothing, which is
    itself the answer.
    """
    link.send("CFG")
    deadline = time.monotonic() + 1.5
    cfg = None
    while time.monotonic() < deadline and cfg is None:
        # The boot banner emits the same line, so accept an earlier one too.
        for line in link.text_lines:
            if line.startswith("INFO,CFG,"):
                cfg = line
                break
        time.sleep(0.05)

    if cfg is None:
        link.firmware_matches_source = False
        r.worst(FAIL, "the board is running firmware older than this sketch")
        r.add("no INFO,CFG reply -- this firmware predates the CFG command")
        r.fix("FLASH THE BOARD FIRST. Upload arduino_mini_base_controller/ to the Uno, "
              "then re-run. Until then the board is not running the constants in this "
              "source tree, and every correction printed here would be computed from "
              "numbers it is not using.")
        return

    fields = {}
    for part in cfg[len("INFO,CFG,"):].split(","):
        key, _, value = part.partition("=")
        fields[key] = value
    r.add(f"firmware reports: {cfg[len('INFO,CFG,'):]}")

    def pair(value: str) -> list[float]:
        return [float(x) for x in value.split("/")]

    differences = []
    try:
        checks = [
            ("counts", pair(fields["counts"]), k.counts_per_rev, "MEASURED_COUNTS_PER_REV"),
            ("enc", pair(fields["enc"]), [float(s) for s in k.encoder_sign], "ENCODER_SIGN"),
            ("mot", pair(fields["mot"]), [float(s) for s in k.motor_direction_sign],
             "MOTOR_DIRECTION_SIGN"),
            ("ysign", pair(fields["ysign"]), [float(s) for s in k.wheel_y_sign],
             "WHEEL_Y_SIGN"),
            ("maxlin", [float(fields["maxlin"])], [k.max_linear_mm_s], "MAX_LINEAR_MM_S"),
            ("maxang", [float(fields["maxang"])], [k.max_angular_deg_s], "MAX_ANGULAR_DEG_S"),
            ("maxwheel", [float(fields["maxwheel"])], [k.max_wheel_mm_s], "MAX_WHEEL_MM_S"),
            ("track", [float(fields["track"])], [k.track_width_mm], "TRACK_WIDTH_MM"),
            ("maxpwm", [float(fields["maxpwm"])], [float(k.max_drive_pwm)], "MAX_DRIVE_PWM"),
        ]
    except (KeyError, ValueError) as exc:
        link.firmware_matches_source = None
        r.worst(WARN, f"could not read the firmware's CFG line ({exc})")
        return

    for _, on_board, in_source, name in checks:
        if any(abs(a - b) > 1e-3 for a, b in zip(on_board, in_source)):
            differences.append(f"{name}: board has {on_board}, source has {in_source}")

    if differences:
        link.firmware_matches_source = False
        r.worst(FAIL, "the board is running different constants from this source tree")
        for line in differences:
            r.add(f"MISMATCH  {line}")
        r.fix("FLASH THE BOARD FIRST. The source tree has been edited since the Uno was "
              "last programmed, so the board is not running the constants listed above. "
              "Upload arduino_mini_base_controller/ and re-run.")
        return

    link.firmware_matches_source = True
    r.add("firmware constants match this source tree")


def test_link(link: ArduinoLink, k: SketchConstants, args) -> Result:
    r = Result("link")
    if not link.wait_for_telemetry(8.0):
        r.worst(FAIL, "no 'T,' telemetry -- wrong port, wrong baud, or the sketch is not running")
        return r

    window = 2.0
    t0 = time.monotonic()
    time.sleep(window)
    samples = link.since(t0)
    rate = len(samples) / window
    r.add(f"telemetry rate {rate:.1f} Hz (expect ~20 Hz)")
    if rate < 12.0:
        r.worst(WARN, f"telemetry only {rate:.1f} Hz")
    else:
        r.worst(PASS, f"link up, {rate:.1f} Hz telemetry")

    if link.ready_banner:
        r.add(f"banner: {link.ready_banner}")
    for line in link.text_lines[-12:]:
        if line.startswith(("ERR,", "INFO,IMU")):
            r.add(f"board says: {line}")

    check_firmware_matches_source(link, k, r)

    last = link.latest()
    if last is None:
        r.worst(FAIL, "telemetry stopped")
        return r

    # setup() latches the e-stop when no IMU answers on I2C, and every VEL is
    # rejected while it is latched -- so an un-cleared latch looks exactly like
    # "the motors do not respond" unless you read this flag.
    if last.estop:
        r.add("E-STOP IS LATCHED -- every VEL will be rejected until it is cleared")
        if ask("clear the latched e-stop now?", args.yes):
            link.send("CLEAR_ESTOP")
            time.sleep(0.3)
            r.add("sent CLEAR_ESTOP")
        else:
            r.worst(FAIL, "e-stop latched, cannot drive")
    if last.watchdog:
        r.add("watchdog currently stopped (expected: nothing is streaming VEL yet)")
    return r


def test_signs(link: ArduinoLink, k: SketchConstants, args) -> Result:
    """Catches the single most dangerous fault in this loop, and the one that
    hides from telemetry.

    A wheel whose encoder reads backwards makes the PID see error = target -
    (-measured), roughly twice the target and never shrinking. The integrator
    winds to its cap and the wheel goes to full PWM and stays there: the robot
    runs flat out no matter how small the command. The calibration sketch
    reports the same fault as WHEEL_MEASURED_BACKWARDS.

    But the wire can only prove the two signs are CONSISTENT, never that they
    are right. Flip both signs on one wheel and it drives backwards while
    reporting a textbook positive speed -- a clean pass, and a robot that turns
    a straight command into a circle. Only an eye on the wheel separates the
    four cases, so this test asks and then reads the truth table:

        physical  measured   meaning                    fix
        forward   positive   correct                    -
        forward   negative   encoder inverted           ENCODER_SIGN
        backward  negative   motor inverted             MOTOR_DIRECTION_SIGN
        backward  positive   both inverted              both

    Mirror-mounted motors land in row three unless their leads are mirrored to
    match: identical wiring drives the two sides in opposite directions, and the
    encoder honestly reports a wheel that is being driven the wrong way.
    """
    r = Result("signs")
    speed = args.sign_speed
    measured_ok = [True, True]

    for direction, sign in (("forward", 1.0), ("reverse", -1.0)):
        print(f"    driving {direction} at {speed:.0f} mm/s -- WATCH THE WHEELS")
        samples = link.drive(sign * speed, 0.0, seconds=2.5, settle=1.5)
        if not samples:
            r.worst(FAIL, "no telemetry while driving")
            return r
        for i in range(2):
            v = mean(s.wheel_mm_s[i] for s in samples)
            pwm = mean(s.wheel_pwm[i] for s in samples)
            peak = max(abs(s.wheel_pwm[i]) for s in samples)
            r.add(f"{direction:7s} {WHEEL[i]:5s} target {sign*speed:+7.1f}  "
                  f"measured {v:+7.1f} mm/s  pwm {pwm:+6.1f} (peak {peak:.0f})")

            if abs(v) < 5.0:
                r.worst(FAIL, f"{WHEEL[i]} wheel does not move {direction}")
                r.fix(f"{WHEEL[i]}: no motion -- check the TB6612 wiring, STBY, and the battery")
            elif (v > 0) != (sign > 0):
                if direction == "forward":
                    measured_ok[i] = False
                r.worst(WARN, f"{WHEEL[i]} wheel measures backwards {direction}")
            elif peak >= k.max_drive_pwm:
                r.worst(WARN, f"{WHEEL[i]} saturated its PWM at only {speed:.0f} mm/s")
                r.fix(f"{WHEEL[i]}: PWM pinned at the ceiling for a low speed target -- the "
                      f"loop is not closing. Suspect the feed-forward fit or a stalled wheel.")

        link.stop()
        time.sleep(0.8)

        # Ask once, on the forward pass, where the sign convention is defined.
        if direction != "forward":
            continue
        drove_backwards = ask_wrong_wheels(
            "    which wheels physically carried the robot BACKWARDS?", args.yes)
        if drove_backwards is None:
            r.add("physical wheel direction NOT observed -- signs are unproven")
            r.worst(WARN, "nobody confirmed which way the wheels actually turned")
            r.fix("Re-run without --yes and watch the wheels. Two inverted signs on one "
                  "wheel look identical to two correct ones in telemetry.")
        else:
            for i in range(2):
                physical_ok = i not in drove_backwards
                verdict, fixes = diagnose_signs(i, physical_ok, measured_ok[i], k)
                r.add(f"{WHEEL[i]:5s} physical {'forward' if physical_ok else 'BACKWARD'}, "
                      f"measured {'positive' if measured_ok[i] else 'NEGATIVE'} -> {verdict}")
                if fixes:
                    r.worst(FAIL, f"{WHEEL[i]} wheel: {verdict}")
                    for line in fixes:
                        r.fix(line)
                else:
                    r.worst(PASS, "both wheels drive and measure the commanded way")

    # Both wheels get the identical setpoint for straight motion, so a large
    # steady-state split between them is a per-wheel calibration problem.
    samples = link.drive(speed, 0.0, seconds=2.5, settle=1.5)
    if samples:
        m1, m2 = (mean(s.wheel_mm_s[i] for s in samples) for i in (0, 1))
        split = abs(m1 - m2)
        r.add(f"straight-line split at {speed:.0f} mm/s: m1 {m1:+.1f}  m2 {m2:+.1f}  "
              f"delta {split:.1f} mm/s")
        if split > max(10.0, 0.15 * speed):
            r.worst(WARN, f"wheels disagree by {split:.1f} mm/s on a straight command")
            r.fix("Per-wheel mismatch on a straight command: re-run the CAL sketch, then "
                  "trim with PIDM before touching the shared PID gains.")
    link.stop()
    return r


def diagnose_signs(index: int, physical_ok: bool, measured_ok: bool,
                   k: SketchConstants) -> tuple[str, list[str]]:
    """Turn one wheel's (physical direction, measured sign) pair into a verdict
    and the exact edits that fix it. See the truth table in test_signs()."""
    wheel = WHEEL[index]
    header = (f"{wheel}: edit MiniRobotGeometry.h, then copy that ONE file over the "
              f"other two sketch folders -- the three copies must stay identical.")

    if physical_ok and measured_ok:
        return "correct", []
    if physical_ok and not measured_ok:
        return "encoder inverted", [
            header,
            f"  const int8_t ENCODER_SIGN[MOTOR_COUNT] = "
            f"{_with(k.encoder_sign, index, -k.encoder_sign[index])};",
            f"  Leave MOTOR_DIRECTION_SIGN alone -- the wheel already drives the right way.",
        ]
    if not physical_ok and not measured_ok:
        return "motor inverted (mirror-mounted wheel)", [
            header,
            f"  const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = "
            f"{_with(k.motor_direction_sign, index, -k.motor_direction_sign[index])};",
            f"  Leave ENCODER_SIGN alone -- that encoder is telling the truth about a "
            f"wheel being driven backwards. Flipping it too cancels the fix and pins "
            f"the wheel at full duty.",
        ]
    return "both signs inverted", [
        header,
        f"  const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = "
        f"{_with(k.motor_direction_sign, index, -k.motor_direction_sign[index])};",
        f"  const int8_t ENCODER_SIGN[MOTOR_COUNT] = "
        f"{_with(k.encoder_sign, index, -k.encoder_sign[index])};",
        f"  This wheel drives backwards while reporting a positive speed, so the loop "
        f"looks healthy and the robot still turns a straight command into a circle. "
        f"Both signs have to move.",
    ]


def _with(values: list[int], index: int, replacement: int) -> str:
    out = list(values)
    out[index] = replacement
    return "{" + ", ".join(str(v) for v in out) + "}"


def test_ladder(link: ArduinoLink, k: SketchConstants, args) -> Result:
    """Closed-loop tracking across the speed range, plus a re-fit of the
    feed-forward line from the PWM the loop actually settled on.

    The re-fit is the part worth reading: it compares the PWM-to-speed
    relationship the robot is living right now against the FF_* constants
    compiled into Config.h. They should agree. If they do not, the fit in
    Config.h is stale and the PID is quietly papering over it.
    """
    r = Result("ladder")
    targets = [t for t in (50.0, 100.0, 150.0, 200.0, 250.0) if t <= k.max_linear_mm_s]
    if not targets:
        targets = [k.max_linear_mm_s]

    points: list[list[tuple[float, float]]] = [[], []]
    r.add(f"{'target':>7} {'L meas':>8} {'L err':>7} {'L pwm':>7} "
          f"{'R meas':>8} {'R err':>7} {'R pwm':>7}")
    for target in targets:
        samples = link.drive(target, 0.0, seconds=3.0, settle=2.0)
        if not samples:
            r.worst(FAIL, "no telemetry while driving")
            return r
        row = [f"{target:7.0f}"]
        for i in range(2):
            v = mean(s.wheel_mm_s[i] for s in samples)
            pwm = mean(abs(s.wheel_pwm[i]) for s in samples)
            err = v - target
            row.append(f"{v:8.1f} {err:+7.1f} {pwm:7.1f}")
            points[i].append((pwm, v))
            if abs(err) > max(8.0, 0.10 * target):
                r.worst(WARN, f"{WHEEL[i]} misses {target:.0f} mm/s by {err:+.1f}")
            if pwm >= k.max_drive_pwm - 1:
                r.worst(FAIL, f"{WHEEL[i]} needs full PWM at {target:.0f} mm/s")
                r.fix(f"{WHEEL[i]}: no PWM headroom left at {target:.0f} mm/s. The robot "
                      f"cannot reach its own MAX_LINEAR_MM_S; lower the limit or find the "
                      f"mechanical drag.")
        r.add(" ".join(row))
        link.stop()
        time.sleep(0.6)

    r.worst(PASS, "closed loop tracks the commanded speed")

    # Least-squares mm/s = slope*pwm + intercept over the ladder, against the
    # forward fit in Config.h.
    for i in range(2):
        pts = [(p, v) for p, v in points[i] if p > 1.0]
        if len(pts) < 3:
            continue
        n = len(pts)
        sx = sum(p for p, _ in pts)
        sy = sum(v for _, v in pts)
        sxx = sum(p * p for p, _ in pts)
        sxy = sum(p * v for p, v in pts)
        denominator = n * sxx - sx * sx
        if abs(denominator) < 1e-9:
            continue
        slope = (n * sxy - sx * sy) / denominator
        intercept = (sy - slope * sx) / n
        compiled = k.ff_slope_fwd[i]
        r.add(f"{WHEEL[i]:5s} observed fit: slope {slope:.4f} intercept {intercept:+.2f} "
              f"| Config.h: slope {compiled:.4f} intercept {k.ff_intercept_fwd[i]:+.2f}")
        if compiled > 0 and abs(slope - compiled) / compiled > 0.15:
            r.worst(WARN, f"{WHEEL[i]} feed-forward slope is off by "
                          f"{100*(slope-compiled)/compiled:+.0f}%")
            r.fix(f"{WHEEL[i]}: the PWM-to-speed line the robot is actually on disagrees "
                  f"with FF_SLOPE_MM_S_PER_PWM_FWD[{i}] in Config.h by "
                  f"{100*(slope-compiled)/compiled:+.0f}%. Re-run the CAL sketch and paste "
                  f"the fresh fit -- do not hand-scale the old one.")
    return r


def test_watchdog(link: ArduinoLink, k: SketchConstants, args) -> Result:
    """Silence on the wire must stop the motors. This is the only thing that
    stops the robot if the Pi, the bridge, or the USB cable dies mid-drive."""
    r = Result("watchdog")
    link.drive(args.sign_speed, 0.0, seconds=2.0, settle=1.5)
    t0 = time.monotonic()
    time.sleep(1.5)   # say nothing at all
    samples = link.since(t0)
    if not samples:
        r.worst(FAIL, "no telemetry")
        return r

    tripped = next((s for s in samples if s.watchdog), None)
    if tripped is None:
        r.worst(FAIL, "watchdog never tripped after 1.5 s of silence")
        r.fix("The command watchdog did not stop the motors. Check COMMAND_WATCHDOG_MS "
              "and that updateControl() is still being called.")
        return r

    delay_ms = (tripped.t_host - t0) * 1000.0
    final = samples[-1]
    r.add(f"watchdog tripped ~{delay_ms:.0f} ms after the last VEL "
          f"(COMMAND_WATCHDOG_MS = 500)")
    r.add(f"after the stop: wheels {final.wheel_mm_s[0]:+.1f} / {final.wheel_mm_s[1]:+.1f} mm/s, "
          f"pwm {final.wheel_pwm[0]:+.0f} / {final.wheel_pwm[1]:+.0f}")
    if max(abs(v) for v in final.wheel_pwm) > 1.0:
        r.worst(FAIL, "motors still driven after the watchdog tripped")
    else:
        r.worst(PASS, f"motors released {delay_ms:.0f} ms after the command stream stopped")
    return r


def test_estop(link: ArduinoLink, k: SketchConstants, args) -> Result:
    r = Result("estop")
    link.drive(args.sign_speed, 0.0, seconds=1.5, settle=1.0)

    before = link.text_count
    link.send("ESTOP")
    time.sleep(0.4)
    latched = link.latest()
    if latched is None or not latched.estop:
        r.worst(FAIL, "ESTOP did not latch")
        link.send("CLEAR_ESTOP")
        return r
    r.add("ESTOP latched, telemetry flag set")
    if max(abs(v) for v in latched.wheel_pwm) > 1.0:
        r.worst(FAIL, "motors still driven while the e-stop is latched")

    # A VEL sent while latched must be refused, not silently queued.
    link.send("VEL,150,0")
    time.sleep(0.3)
    replies = link.text_since(before)
    if any("ESTOP_LATCHED" in line for line in replies):
        r.add("VEL sent while latched was refused with ERR,ESTOP_LATCHED")
    else:
        r.worst(WARN, "no ERR,ESTOP_LATCHED reply to a VEL sent while latched")

    link.send("CLEAR_ESTOP")
    time.sleep(0.4)
    cleared = link.latest()
    if cleared is not None and not cleared.estop:
        r.add("CLEAR_ESTOP released the latch")
        r.worst(PASS, "e-stop latches, suppresses VEL, and clears on request")
    else:
        r.worst(FAIL, "CLEAR_ESTOP did not release the latch")
    return r


def test_scale(link: ArduinoLink, k: SketchConstants, args) -> Result:
    """The one measurement the robot cannot make about itself.

    Everything else in this file compares the robot against its own encoders.
    This compares the encoders against a tape measure, which is the only way to
    catch a wrong MEASURED_COUNTS_PER_REV -- the error that makes the robot
    outrun its command while every number in telemetry looks correct.
    """
    r = Result("scale")
    print()
    print("  FLOOR TEST. Put the robot on the floor with clear space ahead, mark the")
    print(f"  starting point, and let it drive about {args.distance_mm:.0f} mm at "
          f"{args.scale_speed:.0f} mm/s.")
    if not ask("  ready to drive on the floor?", args.yes):
        r.worst(SKIP, "declined")
        return r

    link.send("RESET_ODOM")
    time.sleep(0.4)

    # Drive open-ended on time, not on odometry: odometry distance is the thing
    # under test, so it cannot also be the thing that decides when to stop.
    ramp_allowance = args.scale_speed / 150.0          # LINEAR_ACCEL_MM_S2
    seconds = args.distance_mm / args.scale_speed + ramp_allowance
    start = link.latest()
    link.drive(args.scale_speed, 0.0, seconds=seconds)
    link.stop()
    time.sleep(1.0)
    end = link.latest()
    if start is None or end is None:
        r.worst(FAIL, "no telemetry")
        return r

    odom_mm = math.hypot(end.x_mm - start.x_mm, end.y_mm - start.y_mm)
    ticks = [end.ticks[i] - start.ticks[i] for i in range(2)]
    r.add(f"odometry says it travelled {odom_mm:.1f} mm "
          f"(ticks m1 {ticks[0]}, m2 {ticks[1]})")

    measured = args.measured_mm if args.measured_mm else ask_float(
        "  measure the ACTUAL distance travelled in mm and enter it: ")
    if measured is None or measured <= 0:
        r.worst(SKIP, "no ground-truth measurement given")
        return r

    ratio = odom_mm / measured
    r.add(f"actual {measured:.1f} mm -> odometry/actual = {ratio:.4f}")
    if abs(ratio - 1.0) <= 0.03:
        r.worst(PASS, f"linear scale within {100*abs(ratio-1):.1f}%")
        return r

    faster = "FASTER" if ratio < 1.0 else "SLOWER"
    r.worst(FAIL if abs(ratio - 1.0) > 0.08 else WARN,
            f"robot drives {100*abs(1/ratio - 1):.0f}% {faster} than commanded")

    # odom = ticks * pi*D/N_cfg and truth = ticks * pi*D/N_true, so
    # N_true = N_cfg * odom/truth. Correcting N also rescales every mm/s the
    # feed-forward fit was expressed in, by the same factor.
    corrected = [n * ratio for n in k.counts_per_rev]
    ff_scale = 1.0 / ratio
    r.fix("Ground truth disagrees with the encoders. MEASURED_COUNTS_PER_REV is the "
          "only constant that sets the robot's real speed, so correct it first:")
    r.fix(f"  MiniRobotGeometry.h (copy into BOTH sketch folders):")
    r.fix(f"    const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = "
          f"{{{corrected[0]:.1f}f, {corrected[1]:.1f}f}};")
    r.fix(f"  Config.h feed-forward, all four lines rescaled by {ff_scale:.6f}:")
    r.fix(f"    FF_SLOPE_MM_S_PER_PWM_FWD = {{"
          f"{k.ff_slope_fwd[0]*ff_scale:.4f}f, {k.ff_slope_fwd[1]*ff_scale:.4f}f}};")
    r.fix(f"    FF_INTERCEPT_MM_S_FWD     = {{"
          f"{k.ff_intercept_fwd[0]*ff_scale:.2f}f, {k.ff_intercept_fwd[1]*ff_scale:.2f}f}};")
    r.fix(f"    FF_SLOPE_MM_S_PER_PWM_REV = {{"
          f"{k.ff_slope_rev[0]*ff_scale:.4f}f, {k.ff_slope_rev[1]*ff_scale:.4f}f}};")
    r.fix(f"    FF_INTERCEPT_MM_S_REV     = {{"
          f"{k.ff_intercept_rev[0]*ff_scale:.2f}f, {k.ff_intercept_rev[1]*ff_scale:.2f}f}};")
    r.fix("  Better than rescaling: fix counts/rev, then re-run the CAL sketch so the fit "
          "is measured rather than derived.")
    return r


def test_track(link: ArduinoLink, k: SketchConstants, args) -> Result:
    """Track width only affects rotation, and the IMU is the ground truth for
    it. Run this AFTER 'scale' passes: the wheel speeds this test relies on are
    themselves scaled by MEASURED_COUNTS_PER_REV."""
    r = Result("track")
    print()
    print("  FLOOR TEST. Clear space for an in-place turn. The IMU is the reference,")
    print("  so the robot must be on a surface where the wheels do not slip.")
    if not ask("  ready to turn in place?", args.yes):
        r.worst(SKIP, "declined")
        return r

    rate = min(args.turn_deg_s, k.max_angular_deg_s)
    seconds = 6.0
    link.send("ZERO_YAW")
    time.sleep(0.3)
    samples = link.drive(0.0, rate, seconds=seconds, settle=1.5)
    link.stop()
    if len(samples) < 10:
        r.worst(FAIL, "not enough telemetry")
        return r

    # Unwrap yaw so a turn through +/-180 does not read as a jump.
    yaw = [s.yaw_deg for s in samples]
    unwrapped = [yaw[0]]
    previous_raw = yaw[0]
    for value in yaw[1:]:
        delta = value - previous_raw
        while delta > 180.0:
            delta -= 360.0
        while delta < -180.0:
            delta += 360.0
        unwrapped.append(unwrapped[-1] + delta)
        previous_raw = value
    span = samples[-1].t_host - samples[0].t_host
    if span < 0.5:
        r.worst(FAIL, "sample window too short")
        return r
    measured_rate = (unwrapped[-1] - unwrapped[0]) / span
    gyro_rate = mean(s.wz_deg_s for s in samples)
    r.add(f"commanded {rate:+.1f} deg/s | IMU yaw rate {measured_rate:+.1f} deg/s "
          f"| gyro z {gyro_rate:+.1f} deg/s")

    if measured_rate * rate < 0:
        r.worst(FAIL, "robot turns the wrong way")
        r.fix("Rotation sign is inverted: check IMU_YAW_SIGN in Config.h, and WHEEL_Y_SIGN "
              "in MiniRobotGeometry.h -- if the two motors are recorded on the wrong sides "
              "of the centre line, every turn command turns the other way.")
        return r
    if abs(measured_rate) < 1.0:
        r.worst(FAIL, "robot did not turn")
        return r

    ratio = rate / measured_rate
    corrected = k.track_width_mm * ratio
    r.add(f"TRACK_WIDTH_MM {k.track_width_mm:.1f} -> implied {corrected:.1f} mm")
    if abs(ratio - 1.0) <= 0.06:
        r.worst(PASS, f"turn rate within {100*abs(ratio-1):.1f}% of commanded")
    else:
        faster = "FASTER" if abs(measured_rate) > abs(rate) else "SLOWER"
        r.worst(WARN, f"turns {100*abs(measured_rate/rate - 1):.0f}% {faster} than commanded")
        r.fix(f"Config.h: const float TRACK_WIDTH_MM = {corrected:.1f}f;  "
              f"(measure the contact patches with a ruler as a cross-check -- if the ruler "
              f"and this disagree, the linear scale is still wrong.)")
    return r


TESTS: dict[str, Callable] = {
    "link": test_link,
    "signs": test_signs,
    "ladder": test_ladder,
    "watchdog": test_watchdog,
    "estop": test_estop,
    "scale": test_scale,
    "track": test_track,
}
BENCH_TESTS = ["link", "signs", "ladder", "watchdog", "estop"]
FLOOR_TESTS = ["scale", "track"]


# -----------------------------------------------------------------------------
# Offline model of the firmware's control law
# -----------------------------------------------------------------------------

class ModelPID:
    """Port of WheelPID (Types.h) at lambda == mu == 1.0, which is the classic
    PID fast path the firmware actually runs by default."""

    def __init__(self, kp: float, ki: float, kd: float, max_integral_pwm: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_integral_pwm = max_integral_pwm
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_target = 0.0

    def update(self, target: float, measured: float, dt: float) -> float:
        if abs(target) < 1.0 or target * self.previous_target < 0.0:
            self.integral = 0.0
            self.previous_error = 0.0
        self.previous_target = target
        error = target - measured
        self.integral += error * dt
        limit = self.max_integral_pwm / self.ki if self.ki > 0.0001 else 500.0
        self.integral = max(-limit, min(limit, self.integral))
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


def simulate(target_mm_s: float, k: SketchConstants, counts_true: float,
             encoder_sign: int = 1, seconds: float = 6.0,
             pwm_ceiling: float = 255.0) -> tuple[float, float, float]:
    """Run the firmware's loop against a motor model taken from the logged
    calibration sweep, and report (true mm/s, reported mm/s, PWM) at the end.

    The plant is defined in ticks per second, which is the only thing the
    hardware actually produces -- that keeps the model honest about the
    difference between what the wheel does and what the firmware believes.

    What is modelled: the acceleration ramp, the windowed encoder speed
    estimate, the low-pass, the feed-forward inversion, the breakaway floor and
    the PID with its anti-windup. What is not: load. The stall boost only fires
    against a wheel something is holding, and nothing here holds one, so a run
    that stalls in the model has a control-law fault rather than a torque
    problem.
    """
    # arduino_mini_motor_calibration/cal_result.md, motor 1 forward:
    # mm/s = 6.4639*pwm - 44.31 over PWM 10..72, with a measured hard stall up
    # to PWM 8 and first motion at 10 on the ascending pass. The stall matters:
    # without it the model says a wheel creeps at PWM 8, which no measurement
    # supports and which would hide exactly the dead-zone behaviour
    # FF_BREAKAWAY_PWM exists for.
    #
    # Expressed in ticks/s through the counts/rev the sweep was actually
    # recorded at, so the model does not inherit the assumption under test.
    sweep_counts_per_rev = 520.0
    first_moving_pwm = 10.0

    def ticks_per_second(pwm: float) -> float:
        if pwm < first_moving_pwm:
            return 0.0
        mm_s = 6.4639 * pwm - 44.31
        return max(0.0, mm_s) / (math.pi * k.wheel_diameter_mm / sweep_counts_per_rev)

    dt = 0.01
    mm_per_count_cfg = math.pi * k.wheel_diameter_mm / k.counts_per_rev[0]
    mm_per_count_true = math.pi * k.wheel_diameter_mm / counts_true

    pid = ModelPID(k.pid_kp, k.pid_ki, k.pid_kd, k.max_integral_pwm)
    ramped = 0.0
    filtered = 0.0
    tps = 0.0
    pwm = 0.0
    # The open measurement window: ticks and seconds since the speed estimate
    # was last recomputed (updateWheelSpeed in MotorControl.cpp).
    window_ticks = 0.0
    window_seconds = 0.0

    for _ in range(int(seconds / dt)):
        ramped = min(ramped + 150.0 * dt, target_mm_s)   # LINEAR_ACCEL_MM_S2

        # First-order motor lag toward the open-loop speed for this PWM.
        tps += (ticks_per_second(pwm) - tps) * min(1.0, dt / 0.08)
        ticks = tps * dt

        # The firmware counts whole ticks, and at low speed that quantisation
        # is the whole problem the window exists to solve.
        window_ticks += ticks * encoder_sign
        window_seconds += dt
        if abs(window_ticks) >= 1.0 or window_seconds >= k.speed_window_max_ms / 1000.0:
            raw = (window_ticks * mm_per_count_cfg) / window_seconds
            filtered += 0.35 * (raw - filtered)          # WHEEL_SPEED_FILTER_ALPHA
            window_ticks = 0.0
            window_seconds = 0.0

        slope, intercept = k.ff_slope_fwd[0], k.ff_intercept_fwd[0]
        feed_forward = 0.0
        if ramped >= 1.0 and slope > 0.0:
            feed_forward = max(0.0, min(float(k.max_drive_pwm),
                                        (ramped - intercept) / slope))
        pwm = feed_forward + pid.update(ramped, filtered, dt)
        # Breakaway floor: a commanded wheel that is not turning gets at least
        # enough duty to move (MotorControl.cpp).
        if ramped >= 1.0 and abs(filtered) < k.stall_speed_mm_s:
            pwm = max(pwm, k.ff_breakaway_pwm[0])
        pwm = max(0.0, min(pwm_ceiling, pwm))

    return tps * mm_per_count_true, filtered, pwm


def run_selftest(k: SketchConstants) -> int:
    print("Offline model of the firmware control law -- no hardware involved.")
    print("Plant: motor 1 forward from arduino_mini_motor_calibration/cal_result.md,")
    print("re-expressed in encoder ticks/s so it does not inherit the counts/rev")
    print("assumption under test. Load is not modelled, so the stall boost never fires.\n")
    print(f"Gains read from Config.h: Kp {k.pid_kp}, Ki {k.pid_ki}, Kd {k.pid_kd}, "
          f"integral cap {k.max_integral_pwm:.0f} PWM\n")
    print(f"Config.h now: MEASURED_COUNTS_PER_REV = {k.counts_per_rev[0]:.0f}, "
          f"FF slope {k.ff_slope_fwd[0]:.4f}, intercept {k.ff_intercept_fwd[0]:+.2f}, "
          f"MAX_LINEAR_MM_S = {k.max_linear_mm_s:.0f}\n")

    target = k.max_linear_mm_s
    configured = k.counts_per_rev[0]
    print(f"Commanded {target:.0f} mm/s. 'true' is what a tape measure would see:\n")
    header = f"  {'scenario':<50} {'true mm/s':>10} {'reported':>10} {'pwm':>6}"
    print(header)
    print("  " + "-" * (len(header) - 2))

    for label, counts_true in (
        (f"encoders really are {configured:.0f} counts/rev", configured),
        (f"encoders really are 720 counts/rev", 720.0),
        (f"encoders really are 390 counts/rev (1:15 gearbox)", 390.0),
        (f"encoders really are 440 counts/rev (11 PPR)", 440.0),
    ):
        true_v, reported, pwm = simulate(target, k, counts_true)
        flag = "" if abs(true_v - target) < 0.05 * target else \
               f"   <-- {100*(true_v/target - 1):+.0f}%"
        print(f"  {label:<50} {true_v:10.1f} {reported:10.1f} {pwm:6.1f}{flag}")

    print()
    print("  Same command, but the wheel's encoder counts backwards:")
    true_v, reported, pwm = simulate(target, k, configured, encoder_sign=-1)
    print(f"  {'ENCODER_SIGN wrong on this wheel':<50} {true_v:10.1f} "
          f"{reported:10.1f} {pwm:6.1f}   <-- runs flat out")
    print()
    print("  The reported column is what telemetry, /odom and RViz all show. In every")
    print("  row above it reads close to the command even when the robot is not: the")
    print("  robot cannot see this error, which is why 'scale' needs a tape measure.")
    print()

    ceiling = simulate(target, k, configured, encoder_sign=-1,
                       pwm_ceiling=float(k.max_drive_pwm))[0]
    print(f"  With the same fault but PWM clamped to MAX_DRIVE_PWM ({k.max_drive_pwm}), the")
    print(f"  runaway tops out at {ceiling:.0f} mm/s instead of {true_v:.0f} mm/s -- worth having,")
    print(f"  but it bounds the fault, it does not fix it.")
    return 0


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def report(results: list[Result]) -> int:
    print()
    print("=" * 78)
    print("SUMMARY")
    print("=" * 78)
    for r in results:
        print(f"  {r.status:<5} {r.name:<10} {r.detail}")

    fixes = [(r.name, line) for r in results for line in r.fixes]
    if fixes:
        print()
        print("-" * 78)
        print("WHAT TO CHANGE")
        print("-" * 78)
        for name, line in fixes:
            print(f"  [{name}] {line}")

    if any(r.status == FAIL for r in results):
        return 2
    if any(r.status == WARN for r in results):
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate the mini robot's motor control against its own telemetry "
                    "and against a tape measure.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="tests: " + ", ".join(TESTS) + "   (default: the bench tests)")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--tests", default=",".join(BENCH_TESTS),
                        help="comma-separated, or 'bench', 'floor', 'all'")
    parser.add_argument("--sketch-dir", default=str(SKETCH_DIR),
                        help="folder to read the compiled-in constants from")
    parser.add_argument("--sign-speed", type=float, default=80.0,
                        help="mm/s used by the sign/watchdog/e-stop tests")
    parser.add_argument("--scale-speed", type=float, default=150.0,
                        help="mm/s used by the ground-truth distance run")
    parser.add_argument("--distance-mm", type=float, default=1000.0,
                        help="how far the ground-truth run should drive")
    parser.add_argument("--measured-mm", type=float, default=None,
                        help="actual distance travelled, if you already know it")
    parser.add_argument("--turn-deg-s", type=float, default=60.0,
                        help="deg/s used by the track-width test")
    parser.add_argument("--yes", action="store_true",
                        help="do not prompt before the floor tests")
    parser.add_argument("--selftest", action="store_true",
                        help="offline model only, no serial port needed")
    args = parser.parse_args()

    try:
        constants = load_sketch_constants(Path(args.sketch_dir))
    except (OSError, ValueError) as exc:
        print(f"Cannot read the sketch constants: {exc}", file=sys.stderr)
        return 3

    if args.selftest:
        return run_selftest(constants)

    selection = args.tests.strip().lower()
    if selection == "bench":
        names = BENCH_TESTS
    elif selection == "floor":
        names = ["link"] + FLOOR_TESTS
    elif selection == "all":
        names = BENCH_TESTS + FLOOR_TESTS
    else:
        names = [n.strip() for n in selection.split(",") if n.strip()]
    unknown = [n for n in names if n not in TESTS]
    if unknown:
        print(f"Unknown test(s): {', '.join(unknown)}", file=sys.stderr)
        return 3

    if any(n in FLOOR_TESTS for n in names):
        print("NOTE: this run includes floor tests. Everything else assumes the robot is")
        print("      ON BLOCKS with both wheels free.")
    else:
        print("Put the robot ON BLOCKS -- both wheels must spin free.")
    if not ask("Ready?", args.yes):
        return 3

    try:
        link = ArduinoLink(args.port, args.baud)
    except Exception as exc:
        print(f"Cannot open {args.port}: {exc}", file=sys.stderr)
        print("Hint: scripts/detect_serial_ports.sh works out which adapter is the Arduino.",
              file=sys.stderr)
        return 3

    print(f"Opened {args.port} at {args.baud} baud; waiting for the board to boot...")
    time.sleep(2.0)   # opening the port resets the Uno

    results: list[Result] = []
    try:
        for name in names:
            print()
            print(f"--- {name} " + "-" * (70 - len(name)))
            result = TESTS[name](link, constants, args)
            for note in result.notes:
                print(f"    {note}")
            print(f"    => {result.status}: {result.detail}")
            results.append(result)
            link.stop()
            time.sleep(0.4)
            # A failed link or a wheel that measures backwards invalidates every
            # later number, so stop rather than print misleading ones.
            if result.status == FAIL and name in ("link", "signs"):
                print("\n    Stopping here: later tests would report meaningless numbers "
                      "until this is fixed.")
                break
    except KeyboardInterrupt:
        print("\nInterrupted -- stopping the motors.")
    finally:
        link.close()

    return report(results)


if __name__ == "__main__":
    sys.exit(main())
