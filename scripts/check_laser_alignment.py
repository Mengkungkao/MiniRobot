#!/usr/bin/env python3
"""Measure the LiDAR's yaw offset by driving the robot and watching the scan.

    python3 scripts/check_laser_alignment.py            # 0.2 m forward
    python3 scripts/check_laser_alignment.py --distance 0.4

A laser whose zero does not point along `base_link +x` produces a map that
smears as the robot drives, and nothing in the stack can catch it: the TF tree
is valid, every scan is self-consistent, and SLAM builds a confident wrong map.
The URDF asserts the alignment (`laser_joint` rpy) and nothing measures it.

This measures it, using the robot's own motion as the ground truth so no tape
measure or knowledge of the room is needed.

HOW
When the robot translates by d along bearing phi in the laser's frame, the
range at bearing theta changes by

    dr(theta) ~= -d * cos(theta - phi)

for features far compared with d. That is a cosine across the whole scan, so
fitting

    dr = a*cos(theta) + b*sin(theta)

over every bearing that has a valid return before and after gives both the
direction of travel, atan2(-b, -a), and its length, hypot(a, b). Drive straight
forward and the direction of travel IS +x, so whatever the fit reports is the
laser's yaw offset. Hundreds of beams are averaged, which is why a short move
in a cramped room is enough.

The recovered distance is a free cross-check on odometry: it comes from the
room, not from the wheels.

The robot must be ON THE FLOOR with clear space ahead. It drives itself.
"""
from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Aligner(Node):
    def __init__(self, cmd_topic: str) -> None:
        super().__init__("mini_laser_alignment")
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.scan: LaserScan | None = None
        self.odom: Odometry | None = None
        self.create_subscription(LaserScan, "/scan", self._scan, sensor_qos)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.cmd = self.create_publisher(Twist, cmd_topic, 10)

    def _scan(self, msg): self.scan = msg
    def _odom(self, msg): self.odom = msg

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for_data(self, seconds: float = 10.0) -> bool:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.scan is not None and self.odom is not None:
                return True
        return False

    def average_scan(self, count: int) -> dict[float, float]:
        """Median range per bearing over several scans, to damp the noise the
        fit would otherwise inherit."""
        collected: dict[float, list[float]] = {}
        seen = 0
        last_stamp = None
        while rclpy.ok() and seen < count:
            rclpy.spin_once(self, timeout_sec=0.05)
            scan = self.scan
            if scan is None or scan.header.stamp == last_stamp:
                continue
            last_stamp = scan.header.stamp
            seen += 1
            for i, r in enumerate(scan.ranges):
                if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                    continue
                bearing = scan.angle_min + i * scan.angle_increment
                collected.setdefault(round(math.degrees(bearing)), []).append(r)
        return {b: statistics.median(v) for b, v in collected.items() if len(v) >= count // 2}

    def travelled(self, origin: Odometry) -> float:
        if self.odom is None:
            return 0.0
        return math.hypot(self.odom.pose.pose.position.x - origin.pose.pose.position.x,
                          self.odom.pose.pose.position.y - origin.pose.pose.position.y)

    def stop(self) -> None:
        for _ in range(10):
            self.cmd.publish(Twist())
            self.spin(0.05)

    def drive_forward(self, distance: float, speed: float) -> float:
        origin = self.odom
        twist = Twist()
        twist.linear.x = speed
        deadline = time.monotonic() + distance / speed + 8.0
        moved = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd.publish(twist)
            self.spin(0.05)
            moved = self.travelled(origin)
            if moved >= distance:
                break
        self.stop()
        self.spin(1.5)              # let it settle before the scan is trusted
        return self.travelled(origin)


def fit_translation(before: dict[float, float], after: dict[float, float]):
    """Least-squares fit of dr = a*cos(theta) + b*sin(theta) over shared bearings."""
    shared = sorted(set(before) & set(after))
    # Discard bearings whose range jumped implausibly: an occlusion, a moving
    # object, or a beam that found a different surface entirely. Those are
    # outliers to a model that assumes the same surfaces stayed visible.
    deltas = {b: after[b] - before[b] for b in shared}
    if not deltas:
        return None
    spread = statistics.median([abs(d) for d in deltas.values()]) or 0.01
    usable = {b: d for b, d in deltas.items() if abs(d) < max(0.5, 8.0 * spread)}
    if len(usable) < 30:
        return None

    saa = sab = sbb = sad = sbd = 0.0
    for bearing, delta in usable.items():
        theta = math.radians(bearing)
        ca, sa = math.cos(theta), math.sin(theta)
        saa += ca * ca; sab += ca * sa; sbb += sa * sa
        sad += ca * delta; sbd += sa * delta
    determinant = saa * sbb - sab * sab
    if abs(determinant) < 1e-9:
        return None
    a = (sad * sbb - sbd * sab) / determinant
    b = (sbd * saa - sad * sab) / determinant

    residuals = [delta - (a * math.cos(math.radians(bb)) + b * math.sin(math.radians(bb)))
                 for bb, delta in usable.items()]
    rms = math.sqrt(sum(r * r for r in residuals) / len(residuals))
    return {"bearing_deg": math.degrees(math.atan2(-b, -a)),
            "distance_m": math.hypot(a, b),
            "beams": len(usable), "rms_m": rms}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--distance", type=float, default=0.2, help="metres to drive")
    parser.add_argument("--speed", type=float, default=0.06, help="m/s while driving")
    parser.add_argument("--scans", type=int, default=8, help="scans averaged at each end")
    parser.add_argument("--cmd-topic", default="/cmd_vel_manual")
    args = parser.parse_args()

    rclpy.init()
    node = Aligner(args.cmd_topic)
    try:
        if not node.wait_for_data():
            sys.exit("no /scan or /odom -- is robot.launch.py running?")

        print(f"averaging {args.scans} scans before the move...")
        before = node.average_scan(args.scans)
        start_odom = node.odom

        print(f"driving {args.distance:.2f} m forward at {args.speed:.2f} m/s...")
        moved = node.drive_forward(args.distance, args.speed)
        print(f"odometry says the robot moved {moved:.3f} m")

        print(f"averaging {args.scans} scans after the move...")
        after = node.average_scan(args.scans)

        yaw_change = 0.0
        if start_odom is not None and node.odom is not None:
            def yaw(q):
                return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            yaw_change = math.degrees(yaw(node.odom.pose.pose.orientation)
                                      - yaw(start_odom.pose.pose.orientation))
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    if moved < 0.05:
        sys.exit(f"the robot only moved {moved:.3f} m. If the front stop gate is "
                 f"engaged it zeroes forward commands; clear the space ahead, or "
                 f"lower front_stop_distance in base.yaml.")

    result = fit_translation(before, after)
    if result is None:
        sys.exit("not enough matching bearings before and after to fit a translation. "
                 "The room may be too featureless, or too much of it moved.")

    print()
    print("=" * 66)
    print(f"  beams used in the fit : {result['beams']}")
    print(f"  fit residual (RMS)    : {result['rms_m'] * 100:.1f} cm")
    print(f"  yaw change while driving: {yaw_change:+.1f} deg (should be near 0)")
    print()
    print(f"  distance from the room  : {result['distance_m']:.3f} m")
    print(f"  distance from the wheels: {moved:.3f} m")
    scale_error = 100.0 * (moved - result["distance_m"]) / max(result["distance_m"], 1e-6)
    print(f"  odometry scale error    : {scale_error:+.1f}%")
    print()
    offset = math.degrees(math.atan2(math.sin(math.radians(result["bearing_deg"])),
                                     math.cos(math.radians(result["bearing_deg"]))))
    print(f"  the robot travelled toward bearing {offset:+.1f} deg in the LASER frame")
    print(f"  it was commanded straight forward, which is bearing 0")
    print()
    print(f"  ==> LASER YAW OFFSET: {offset:+.1f} deg")
    print("=" * 66)
    print()
    if abs(offset) < 5.0:
        print("  Aligned. laser_joint needs no yaw correction.")
    else:
        print("  MISALIGNED. The laser frame is rotated relative to the direction")
        print("  the robot drives, so every scan is placed at the wrong bearing and")
        print("  the map will smear as the robot moves. Correct it in the URDF:")
        print()
        print("    ros2_ws/src/mini_robot/urdf/mini_robot.urdf.xacro, laser_joint:")
        print(f"      <origin xyz=\"...\" rpy=\"0 0 {math.radians(offset):+.4f}\"/>")
        print()
        print("  Fix it there, not in the driver: the URDF is where the mounting is")
        print("  described, and SLAM, Nav2 and the costmaps all read it from TF.")
    if abs(scale_error) > 10.0:
        print()
        print(f"  Also: the wheels and the room disagree about distance by "
              f"{scale_error:+.0f}%.")
        print("  Check MEASURED_COUNTS_PER_REV and WHEEL_DIAMETER_MM.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
