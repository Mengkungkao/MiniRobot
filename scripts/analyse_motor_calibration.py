#!/usr/bin/env python3
"""Turn a raw CAL log from arduino_mini_motor_calibration into a report.

    python3 scripts/analyse_motor_calibration.py cal_raw.log \
        -o arduino_mini_motor_calibration/cal_result.md

The sketch fits the line on-board and prints the four constants ready to paste,
which is all the firmware needs. It cannot keep the points it fitted, though --
2 KB of SRAM does not hold 280 samples -- so everything except the answer is
gone the moment the serial monitor scrolls. This keeps the points, refits them
off-board as a cross-check, and writes the whole thing out as markdown with the
data in a form anything can plot.

What it adds beyond the sketch's own output:

- every sample preserved, tagged with the pass it came from, so the ascending
  and descending legs can be compared (a gearmotor settles at a slightly
  different speed at the same PWM depending on which side it approached from)
- an independent least-squares fit, to catch an on-board fit that disagrees
- residuals, which say whether "linear" is actually true or just assumed
- measured breakaway against the fit's extrapolated dead zone, the gap that
  FF_BREAKAWAY_PWM in the firmware exists to cover
"""
from __future__ import annotations

import argparse
import re
import sys
from datetime import date
from pathlib import Path

DIRECTION_NAME = {"F": "forward", "R": "reverse"}


class Sample:
    __slots__ = ("motor", "direction", "pwm", "speed", "used", "pass_index")

    def __init__(self, motor, direction, pwm, speed, used, pass_index):
        self.motor = motor
        self.direction = direction
        self.pwm = pwm
        self.speed = speed
        self.used = used
        self.pass_index = pass_index


def parse_log(text: str):
    """Pull samples, on-board fits and session settings out of a raw log.

    Pass number is recovered from the order the PWM values arrive in rather
    than from the log, which does not record it: the sketch sweeps up, then
    down, so the pass advances every time the direction of travel reverses.
    """
    samples: list[Sample] = []
    board_fits: dict[tuple[int, str], dict] = {}
    meta: dict[str, str] = {}
    signs: dict[int, tuple[int, int]] = {}
    paste: list[str] = []
    paste_by_name: dict[str, int] = {}
    elapsed = None

    # Per (motor, direction): the previous PWM and the current pass index.
    walk: dict[tuple[int, str], list] = {}

    for line in text.splitlines():
        line = line.strip()

        if line.startswith("C,"):
            _, motor, direction, pwm, speed, verdict = line.split(",")
            motor, pwm, speed = int(motor), int(pwm), float(speed)
            key = (motor, direction)
            state = walk.setdefault(key, [None, 0])
            if state[0] is not None and pwm == state[0]:
                # Same PWM twice running is the turn at the top of the sweep.
                state[1] += 1
            state[0] = pwm
            samples.append(Sample(motor, direction, pwm, speed,
                                  verdict == "USED", state[1]))

        elif line.startswith("R,") and not line.startswith("R,motor"):
            parts = line.split(",")
            if len(parts) == 9:
                board_fits[(int(parts[1]), parts[2])] = {
                    "slope": float(parts[3]), "intercept": float(parts[4]),
                    "r2": float(parts[5]), "samples": int(parts[6]),
                    "deadzone": float(parts[7]), "speed_at_max": float(parts[8]),
                }

        elif line.startswith("SIGN,"):
            parts = line.split(",")
            signs[int(parts[1])] = (int(parts[3]), int(parts[5]))

        elif line.startswith("INFO,GEOMETRY,"):
            meta["geometry"] = line[len("INFO,GEOMETRY,"):]
        elif line.startswith("INFO,CONFIG,"):
            meta["config"] = line[len("INFO,CONFIG,"):]
        elif line.startswith("const float FF_"):
            # RESULT reprints the paste block, so the same four lines usually
            # appear twice in one log. Keep the last value seen for each
            # constant, in first-seen order.
            name = line.split("[", 1)[0]
            if name in paste_by_name:
                paste[paste_by_name[name]] = line
            else:
                paste_by_name[name] = len(paste)
                paste.append(line)
        elif line.startswith("### elapsed"):
            elapsed = line[len("### elapsed"):].strip()
        elif line.startswith("INFO,SLOPE_MISMATCH_"):
            meta.setdefault("mismatch", []).append(line[len("INFO,"):])

    return samples, board_fits, meta, signs, paste, elapsed


def least_squares(points: list[tuple[float, float]]):
    """Ordinary least squares of speed on PWM, plus r^2 and residuals."""
    n = len(points)
    if n < 2:
        return None
    sx = sum(x for x, _ in points)
    sy = sum(y for _, y in points)
    sxx = sum(x * x for x, _ in points)
    sxy = sum(x * y for x, y in points)
    denominator = n * sxx - sx * sx
    if abs(denominator) < 1e-9:
        return None
    slope = (n * sxy - sx * sy) / denominator
    intercept = (sy - slope * sx) / n
    mean_y = sy / n
    ss_total = sum((y - mean_y) ** 2 for _, y in points)
    residuals = [y - (slope * x + intercept) for x, y in points]
    ss_residual = sum(r * r for r in residuals)
    r2 = 1.0 - ss_residual / ss_total if ss_total > 0 else 1.0
    rms = (ss_residual / n) ** 0.5
    return {"slope": slope, "intercept": intercept, "r2": r2, "n": n,
            "rms": rms, "max_abs": max(abs(r) for r in residuals),
            "residuals": residuals}


def ascii_plot(series: dict[str, list[tuple[int, float]]], width=64, height=18):
    """A quick visual check that renders anywhere markdown does.

    Not a substitute for the CSV below -- it is here so a wrong-looking curve is
    obvious without leaving the file.
    """
    points = [p for pts in series.values() for p in pts]
    if not points:
        return "(no data)"
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    x_lo, x_hi = min(xs), max(xs)
    y_lo, y_hi = min(min(ys), 0.0), max(ys)
    if x_hi == x_lo or y_hi == y_lo:
        return "(degenerate range)"

    marks = {name: chr(ord("1") + i) for i, name in enumerate(series)}
    # Track which series have landed on each cell. On this robot all four
    # curves sit almost on top of each other, so a grid that simply overwrites
    # would show whichever series was drawn last and quietly imply the others
    # are somewhere else. '*' means "more than one curve here", which is the
    # honest and, as it happens, the interesting answer.
    occupied: dict[tuple[int, int], set] = {}
    for name, pts in series.items():
        for x, y in pts:
            col = int((x - x_lo) / (x_hi - x_lo) * (width - 1))
            row = height - 1 - int((y - y_lo) / (y_hi - y_lo) * (height - 1))
            occupied.setdefault((row, col), set()).add(name)

    grid = [[" "] * width for _ in range(height)]
    for (row, col), names in occupied.items():
        grid[row][col] = "*" if len(names) > 1 else marks[next(iter(names))]

    lines = []
    for i, row in enumerate(grid):
        value = y_hi - (y_hi - y_lo) * i / (height - 1)
        lines.append(f"{value:6.0f} |" + "".join(row))
    lines.append("       +" + "-" * width)
    lines.append(f"        {x_lo:<{width - 6}}{x_hi:>6}")
    lines.append("        PWM")
    lines.append("")
    lines.append("  " + "   ".join(f"{m} = {name}" for name, m in marks.items()))
    lines.append("  * = two or more curves in the same cell")
    return "\n".join(lines)


def build_report(samples, board_fits, meta, signs, paste, elapsed, source_name):
    out: list[str] = []
    w = out.append

    keys = sorted({(s.motor, s.direction) for s in samples})
    by_key = {k: [s for s in samples if (s.motor, s.direction) == k] for k in keys}
    own_fits = {k: least_squares([(s.pwm, s.speed) for s in by_key[k] if s.used])
                for k in keys}

    w("# Motor calibration results")
    w("")
    w(f"Run on {date.today().isoformat()} with the robot **on blocks**, both wheels")
    w("free. Produced by `arduino_mini_motor_calibration` and written up by")
    w("`scripts/analyse_motor_calibration.py`.")
    w("")
    w("To reproduce:")
    w("")
    w("```bash")
    w("bash scripts/build_arduino.sh arduino_mini_motor_calibration")
    w("python3 scripts/flash_arduino.py build/arduino_mini_motor_calibration/*.hex")
    w("# then over serial at 500000 baud:")
    w("#   RANGE,4,72   STEP,2   DWELL,400,600   PASSES,2   CAL")
    w(f"python3 scripts/analyse_motor_calibration.py {source_name} -o cal_result.md")
    w("```")
    w("")

    w("## Session")
    w("")
    w("| | |")
    w("| --- | --- |")
    if "geometry" in meta:
        w(f"| geometry | `{meta['geometry']}` |")
    if "config" in meta:
        w(f"| sweep | `{meta['config']}` |")
    for motor in sorted(signs):
        enc, mot = signs[motor]
        w(f"| motor {motor} signs | `ENCODER_SIGN {enc:+d}`, `MOTOR_DIRECTION_SIGN {mot:+d}` |")
    w(f"| samples | {len(samples)} ({sum(1 for s in samples if s.used)} used, "
      f"{sum(1 for s in samples if not s.used)} below the stall threshold) |")
    if elapsed:
        w(f"| run time | {elapsed} |")
    w("")
    w("The signs above must match what the base controller reports for `CFG`. They")
    w("decide what every number below **means**: a sweep labelled forward that")
    w("actually drove the wheel backwards fits a line to the wrong quadrant.")
    w("")

    w("## Fits")
    w("")
    w("`speed_mm_s = slope * pwm + intercept`, fitted over the points above the")
    w("stall threshold. Two fits are shown per row: the one the sketch computed")
    w("on-board and pasted into `Config.h`, and an independent least-squares fit")
    w("of the same points done here. They should agree to the last digit; a")
    w("disagreement means one of the two is reading the data wrong.")
    w("")
    w("| motor | direction | slope | intercept | r² | n | dead zone PWM | mm/s at PWM 210 |")
    w("| --- | --- | --- | --- | --- | --- | --- | --- |")
    for key in keys:
        motor, direction = key
        board = board_fits.get(key)
        if board:
            w(f"| {motor} | {DIRECTION_NAME[direction]} (board) | {board['slope']:.4f} | "
              f"{board['intercept']:+.2f} | {board['r2']:.4f} | {board['samples']} | "
              f"{board['deadzone']:.1f} | {board['speed_at_max']:.0f} |")
        fit = own_fits[key]
        if fit:
            dead = -fit["intercept"] / fit["slope"] if fit["slope"] else float("nan")
            w(f"| {motor} | {DIRECTION_NAME[direction]} (refit) | {fit['slope']:.4f} | "
              f"{fit['intercept']:+.2f} | {fit['r2']:.4f} | {fit['n']} | "
              f"{dead:.1f} | {fit['slope'] * 210 + fit['intercept']:.0f} |")
    w("")

    w("## Where the line stops describing the motor")
    w("")
    w("The fit's dead zone is where the fitted line crosses zero speed. The")
    w("measured breakaway is the lowest PWM that actually produced motion. The")
    w("gap between them is real and is why `FF_BREAKAWAY_PWM` exists in the")
    w("firmware: inside it the bridge is energised and the wheel is not turning.")
    w("")
    w("| motor | direction | fit says zero at PWM | last stalled PWM | first moving PWM | first speed | verdict |")
    w("| --- | --- | --- | --- | --- | --- | --- |")
    contested_any = False
    for key in keys:
        motor, direction = key
        moving = sorted((s for s in by_key[key] if s.used), key=lambda s: s.pwm)
        stalled = sorted((s for s in by_key[key] if not s.used), key=lambda s: s.pwm)
        fit = own_fits[key]
        dead = f"{-fit['intercept'] / fit['slope']:.1f}" if fit and fit["slope"] else "-"
        last_stall = stalled[-1].pwm if stalled else None
        first_move = moving[0].pwm if moving else None
        # The two passes can disagree at the same PWM: the ascending leg has to
        # break the wheel out of stiction, the descending leg arrives with it
        # already turning and keeps it going. Where that happens, breakaway is
        # not a single number and the firmware floor has to sit above it.
        contested = last_stall is not None and first_move is not None and last_stall >= first_move
        contested_any = contested_any or contested
        verdict = "contested — see below" if contested else "clean"
        w(f"| {motor} | {DIRECTION_NAME[direction]} | {dead} | "
          f"{last_stall if last_stall is not None else 'none'} | "
          f"{first_move if first_move is not None else 'never moved'} | "
          f"{moving[0].speed:.1f} mm/s | {verdict} |" if moving else
          f"| {motor} | {DIRECTION_NAME[direction]} | {dead} | {last_stall} | "
          f"never moved | - | {verdict} |")
    w("")
    if contested_any:
        w("**Contested** means the two passes disagreed at the same PWM: going up,")
        w("the wheel was still stalled; coming down, it was still turning. That is")
        w("stiction, and it means breakaway is a band rather than a number. The")
        w("firmware's `FF_BREAKAWAY_PWM` has to sit at the top of that band, because")
        w("a wheel starting from rest is always on the ascending side of it.")
        w("")

    w("## Breakaway floor for the firmware")
    w("")
    w("`FF_BREAKAWAY_PWM` in `arduino_mini_base_controller/Config.h` is the output")
    w("floor applied to a wheel that is commanded and not turning. The number that")
    w("belongs there is the **ascending** pass's first moving PWM: a wheel starting")
    w("from rest has to break stiction, which is the ascending case. Taking it from")
    w("the descending pass, or from the fit's extrapolated dead zone, gives a floor")
    w("below the PWM at which the wheel actually starts — and a wheel commanded into")
    w("that gap draws current and does nothing until the stall boost rescues it.")
    w("")
    w("It is one value per wheel, not per direction, so each wheel takes its worse")
    w("direction.")
    w("")
    w("| motor | forward | reverse | floor needed |")
    w("| --- | --- | --- | --- |")
    floors = {}
    for motor in sorted({m for m, _ in keys}):
        per_direction = {}
        for direction in ("F", "R"):
            ascending = sorted((x for x in by_key.get((motor, direction), [])
                                if x.pass_index == 0 and x.used), key=lambda x: x.pwm)
            per_direction[direction] = ascending[0].pwm if ascending else None
        values = [v for v in per_direction.values() if v is not None]
        floors[motor] = max(values) if values else None
        w(f"| {motor} | "
          f"{per_direction['F'] if per_direction['F'] is not None else '-'} | "
          f"{per_direction['R'] if per_direction['R'] is not None else '-'} | "
          f"**{floors[motor] if floors[motor] is not None else '-'}** |")
    w("")
    if all(v is not None for v in floors.values()):
        values = ", ".join(f"{floors[m]:.1f}f" for m in sorted(floors))
        w("```cpp")
        w(f"const float FF_BREAKAWAY_PWM[MOTOR_COUNT] = {{{values}}};")
        w("```")
        w("")
        if len(set(floors.values())) > 1:
            w("The two wheels differ, which is the argument against a single shared")
            w("floor: setting both to the higher value makes the easier wheel lurch at")
            w("every start, and both to the lower leaves the stiffer one dead.")
            w("")

    w("## Residuals")
    w("")
    w("How far the real motor departs from the straight line, over the fitted")
    w("range. Small and unstructured means linear is a fair model; a large or")
    w("systematically curved residual means it is not, and the feed-forward will")
    w("be wrong at the ends of the range even though r² looks perfect.")
    w("")
    w("| motor | direction | RMS residual | worst residual | worst as % of range |")
    w("| --- | --- | --- | --- | --- |")
    for key in keys:
        fit = own_fits[key]
        if not fit:
            continue
        span = max(s.speed for s in by_key[key]) - min(s.speed for s in by_key[key] if s.used)
        worst_pct = 100.0 * fit["max_abs"] / span if span else 0.0
        w(f"| {key[0]} | {DIRECTION_NAME[key[1]]} | {fit['rms']:.2f} mm/s | "
          f"{fit['max_abs']:.2f} mm/s | {worst_pct:.1f}% |")
    w("")

    w("## Hysteresis: sweeping up vs sweeping down")
    w("")
    w("Each direction is swept twice, ascending then descending. A gearmotor")
    w("settles at a slightly different speed at the same PWM depending on which")
    w("side it approached from, and the fit lands in the middle of that loop.")
    w("A large gap here is backlash or a dragging gearbox, not measurement noise.")
    w("")
    w("| motor | direction | mean up − down | largest gap | at PWM |")
    w("| --- | --- | --- | --- | --- |")
    for key in keys:
        up = {s.pwm: s.speed for s in by_key[key] if s.pass_index == 0}
        down = {s.pwm: s.speed for s in by_key[key] if s.pass_index == 1}
        shared = sorted(set(up) & set(down))
        if not shared:
            continue
        gaps = [(up[p] - down[p], p) for p in shared]
        mean_gap = sum(g for g, _ in gaps) / len(gaps)
        worst, worst_pwm = max(gaps, key=lambda g: abs(g[0]))
        w(f"| {key[0]} | {DIRECTION_NAME[key[1]]} | {mean_gap:+.2f} mm/s | "
          f"{worst:+.2f} mm/s | {worst_pwm} |")
    w("")

    w("## Comparisons that matter to the robot")
    w("")
    for motor in sorted({m for m, _ in keys}):
        f_fit, r_fit = own_fits.get((motor, "F")), own_fits.get((motor, "R"))
        if f_fit and r_fit:
            diff = 100.0 * abs(f_fit["slope"] - r_fit["slope"]) / max(f_fit["slope"], r_fit["slope"])
            w(f"- **Motor {motor}, forward vs reverse:** slopes differ by {diff:.1f}%. "
              f"This is why the firmware keeps a separate fit per direction; one "
              f"averaged line would under-drive one way and over-drive the other.")
    for direction in ("F", "R"):
        a, b = own_fits.get((1, direction)), own_fits.get((2, direction))
        if a and b:
            diff = 100.0 * abs(a["slope"] - b["slope"]) / max(a["slope"], b["slope"])
            w(f"- **Motor 1 vs motor 2, {DIRECTION_NAME[direction]}:** slopes differ by "
              f"{diff:.1f}%. Same command, different PWM needed — which is what the "
              f"per-wheel PID trims out, and why a shared fit would make the robot curve.")
    w("")

    w("## Curve")
    w("")
    w("```text")
    series = {}
    for key in keys:
        label = f"motor {key[0]} {DIRECTION_NAME[key[1]]}"
        series[label] = [(s.pwm, s.speed) for s in by_key[key]]
    w(ascii_plot(series))
    w("```")
    w("")

    w("## Speed at each PWM step")
    w("")
    w("Both passes averaged, so this is the line the fit actually sees. Blank")
    w("where the wheel did not turn.")
    w("")
    w("| PWM | m1 forward | m1 reverse | m2 forward | m2 reverse |")
    w("| ---: | ---: | ---: | ---: | ---: |")
    all_pwm = sorted({s.pwm for s in samples})
    for pwm in all_pwm:
        cells = []
        for key in ((1, "F"), (1, "R"), (2, "F"), (2, "R")):
            matching = [s.speed for s in by_key.get(key, []) if s.pwm == pwm]
            if not matching:
                cells.append("")
            else:
                mean = sum(matching) / len(matching)
                cells.append("stall" if abs(mean) < 1e-6 else f"{mean:.1f}")
        w(f"| {pwm} | " + " | ".join(cells) + " |")
    w("")

    w("## Every sample")
    w("")
    w("One row per measurement, in the order they were taken. `pass` 0 is the")
    w("ascending sweep and 1 the descending one. This is the block to load into")
    w("a plotting tool.")
    w("")
    w("```csv")
    w("motor,direction,pass,pwm,speed_mm_s,used")
    for s in samples:
        w(f"{s.motor},{DIRECTION_NAME[s.direction]},{s.pass_index},{s.pwm},"
          f"{s.speed:.2f},{int(s.used)}")
    w("```")
    w("")

    if paste:
        w("## Paste into `arduino_mini_base_controller/Config.h`")
        w("")
        w("```cpp")
        for line in paste:
            w(line)
        w("```")
        w("")
        w("After pasting, rebuild and reflash the base controller, then confirm the")
        w("board is running them:")
        w("")
        w("```bash")
        w("bash scripts/build_arduino.sh arduino_mini_base_controller")
        w("python3 scripts/flash_arduino.py build/arduino_mini_base_controller/*.hex")
        w("python3 scripts/validate_motor_control.py --tests link,ladder")
        w("```")
        w("")

    return "\n".join(out) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("log", type=Path, help="raw serial log from a CAL run")
    parser.add_argument("-o", "--output", type=Path, help="markdown file to write "
                                                          "(default: stdout)")
    args = parser.parse_args()

    if not args.log.is_file():
        sys.exit(f"no such file: {args.log}")

    parsed = parse_log(args.log.read_text())
    if not parsed[0]:
        sys.exit(f"{args.log} contains no 'C,' sample lines -- was CAL actually run?")

    report = build_report(*parsed, source_name=args.log.name)
    if args.output:
        args.output.write_text(report)
        print(f"wrote {args.output} ({len(parsed[0])} samples)")
    else:
        print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
