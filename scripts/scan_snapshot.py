#!/usr/bin/env python3
"""Print one /scan as a top-down ASCII map, to check where the LiDAR thinks
forward is.

    ros2 run ... # or just:
    python3 scripts/scan_snapshot.py
    python3 scripts/scan_snapshot.py --max-range 2.0 --frame base_link

A laser mounted with its zero pointing somewhere other than `base_link +x`
produces a map that smears as the robot drives, and nothing in the stack can
detect it: every scan is self-consistent, the TF tree is valid, and SLAM
happily builds a wrong map out of correctly-rotated garbage. The only check is
to look at a scan and compare it with the room you are standing in.

That is what this prints: the scan drawn from above, with +x (the direction a
positive cmd_vel.linear.x drives, and the end the LiDAR is supposed to face) at
the TOP of the picture and +y (port) to the LEFT, which is how RViz shows it.
Compare the shape against the actual room. If the wall that is really in front
of the robot appears on the left of the picture, the laser frame is yawed by
90 degrees and `laser_joint` in the URDF is what needs fixing -- not the driver
and not SLAM.
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

# Bearings worth naming, in degrees, with +ccw from straight ahead.
NAMED_BEARINGS = [("ahead", 0.0), ("port bow (+45)", 45.0), ("port (+90)", 90.0),
                  ("port quarter (+135)", 135.0), ("astern (180)", 180.0),
                  ("stbd quarter (-135)", -135.0), ("starboard (-90)", -90.0),
                  ("stbd bow (-45)", -45.0)]


def collect(topic: str, timeout_s: float) -> LaserScan | None:
    rclpy.init()
    node = Node("mini_scan_snapshot")
    # The driver publishes best-effort, like most sensor sources.
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST)
    holder: list[LaserScan] = []
    node.create_subscription(LaserScan, topic, holder.append, qos)
    end = node.get_clock().now().nanoseconds * 1e-9 + timeout_s
    try:
        while rclpy.ok() and not holder:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds * 1e-9 > end:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return holder[-1] if holder else None


def beam_angle(scan: LaserScan, index: int) -> float:
    return scan.angle_min + index * scan.angle_increment


def range_at(scan: LaserScan, bearing_deg: float) -> float:
    """Nearest valid return within half a degree of a bearing."""
    target = math.radians(bearing_deg)
    best, best_error = float("inf"), float("inf")
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r):
            continue
        # Shortest angular distance, so 179 and -179 are close together.
        error = abs(math.atan2(math.sin(beam_angle(scan, i) - target),
                               math.cos(beam_angle(scan, i) - target)))
        if error < best_error:
            best, best_error = r, error
    return best if best_error < math.radians(1.5) else float("inf")


def draw(scan: LaserScan, max_range: float, width: int, height: int) -> str:
    # Screen: +x (forward) up, +y (port) left -- the RViz top-down view.
    # Character cells are about twice as tall as wide, so the x scale is
    # halved to keep the picture roughly square.
    grid = [[" "] * width for _ in range(height)]
    cx, cy = width // 2, height // 2

    plotted = 0
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r > max_range or r < scan.range_min:
            continue
        angle = beam_angle(scan, i)
        x, y = r * math.cos(angle), r * math.sin(angle)
        col = cx - int(round(y / max_range * (width // 2)))
        row = cy - int(round(x / max_range * (height // 2)))
        if 0 <= row < height and 0 <= col < width:
            grid[row][col] = "#"
            plotted += 1

    # Axes through the robot, drawn under the returns.
    for row in range(height):
        if grid[row][cx] == " ":
            grid[row][cx] = "|"
    for col in range(width):
        if grid[cy][col] == " ":
            grid[cy][col] = "-"
    grid[cy][cx] = "R"
    # Mark forward unmistakably: this is the whole point of the picture.
    if grid[0][cx] in ("|", " "):
        grid[0][cx] = "^"

    lines = ["".join(row) for row in grid]
    lines.insert(0, f"  +x / FORWARD / the LiDAR end is UP  ({max_range:.1f} m to an edge)")
    lines.append(f"  R = robot   # = return   ^ = forward   {plotted} beams drawn")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--max-range", type=float, default=3.0,
                        help="metres from the robot to the edge of the picture")
    parser.add_argument("--width", type=int, default=79)
    parser.add_argument("--height", type=int, default=31)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    scan = collect(args.topic, args.timeout)
    if scan is None:
        sys.exit(f"no message on {args.topic} within {args.timeout:.0f} s "
                 f"-- is the LiDAR node running?")

    valid = [r for r in scan.ranges if math.isfinite(r)]
    print(f"frame_id      : {scan.header.frame_id}")
    print(f"beams         : {len(scan.ranges)} over "
          f"{math.degrees(scan.angle_max - scan.angle_min):.0f} deg, "
          f"{len(valid)} valid ({100 * len(valid) / max(1, len(scan.ranges)):.0f}%)")
    print(f"range limits  : {scan.range_min:.2f} .. {scan.range_max:.2f} m")
    print()
    print("Range by bearing (+ is counter-clockwise / to port):")
    for name, bearing in NAMED_BEARINGS:
        r = range_at(scan, bearing)
        print(f"  {name:<22} {'inf' if math.isinf(r) else f'{r:.2f} m'}")

    if valid:
        nearest_index = min((i for i, r in enumerate(scan.ranges) if math.isfinite(r)),
                            key=lambda i: scan.ranges[i])
        print()
        print(f"nearest return: {scan.ranges[nearest_index]:.2f} m at "
              f"{math.degrees(beam_angle(scan, nearest_index)):+.0f} deg")
    print()
    print(draw(scan, args.max_range, args.width, args.height))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
