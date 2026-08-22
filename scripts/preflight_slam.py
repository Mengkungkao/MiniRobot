#!/usr/bin/env python3
"""One-shot readiness check for the mini robot, run before starting SLAM.

Answers the four questions that decide whether a mapping run is worth starting,
in the order that makes a failure easy to place:

  graph    are the nodes up, are the topics publishing, is the base healthy
  sensors  do the IMU and LiDAR return numbers that make physical sense
  model    does the robot description match the robot, and is the TF tree whole
  motion   ON BLOCKS: does a command reach the wheels, and do they obey it

Everything before 'motion' is read-only and safe with the robot on the floor.
'motion' spins the wheels and asks first.

  python3 scripts/preflight_slam.py                 all four phases
  python3 scripts/preflight_slam.py --no-motion     read-only, skip the driving
  python3 scripts/preflight_slam.py --yes           do not prompt before driving

Run it from either machine -- it only needs the ROS graph, not the serial port,
so it checks the stack the way SLAM will actually see it. For faults below the
ROS layer (wheel signs, feed-forward, encoder scale) use
scripts/validate_motor_control.py on the Pi, which talks to the Arduino
directly.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"
SEVERITY = {SKIP: 0, PASS: 1, WARN: 2, FAIL: 3}

WHEEL_RADIUS_M = 0.0235                 # 47 mm diameter
RAD_S_TO_MM_S = WHEEL_RADIUS_M * 1000.0
STANDARD_GRAVITY = 9.80665

# Names the URDF is expected to define. Mismatches here break RViz and, for the
# laser frame, silently misplace every scan SLAM receives.
EXPECTED_LINKS = {"base_footprint", "base_link", "left_wheel_link",
                  "right_wheel_link", "caster_link", "imu_link", "laser"}
EXPECTED_JOINTS = {"base_footprint_joint", "left_wheel_joint",
                   "right_wheel_joint", "caster_joint", "imu_joint", "laser_joint"}
# Joints the base actually reports, which must be a subset of the URDF's.
DRIVEN_JOINTS = ["left_wheel_joint", "right_wheel_joint"]

# Expected publish rates. Lower bounds, generous enough not to fire on jitter.
MIN_RATES = {"/scan": 4.0, "/imu/data": 10.0, "/odom": 10.0,
             "/joint_states": 10.0, "/diagnostics": 0.5}

STREAM_HZ = 20.0
RUNAWAY_MM_S = 300.0
FRONT_STOP_M = 0.30                     # cmd_mux front_stop_distance


# -----------------------------------------------------------------------------
# Results
# -----------------------------------------------------------------------------

@dataclass
class Check:
    name: str
    status: str
    detail: str = ""
    fix: str = ""


@dataclass
class Phase:
    name: str
    checks: list[Check] = field(default_factory=list)

    def add(self, name: str, status: str, detail: str = "", fix: str = "") -> None:
        self.checks.append(Check(name, status, detail, fix))

    @property
    def status(self) -> str:
        if not self.checks:
            return SKIP
        return max((c.status for c in self.checks), key=lambda s: SEVERITY[s])


class Probe:
    """Counts messages and keeps the newest, for rate and content checks."""

    def __init__(self) -> None:
        self.count = 0
        self.last = None
        self.first_time = None
        self.last_time = None

    def hit(self, msg) -> None:
        now = time.monotonic()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.count += 1
        self.last = msg

    def rate(self) -> float:
        # Measure over the span actually observed, not the whole window: a topic
        # that starts late should not be reported as slow.
        if self.count < 2 or self.first_time is None:
            return 0.0
        span = self.last_time - self.first_time
        return (self.count - 1) / span if span > 0 else 0.0


# -----------------------------------------------------------------------------
# Node
# -----------------------------------------------------------------------------

class Preflight(Node):
    def __init__(self) -> None:
        super().__init__("mini_preflight")

        self.scan = Probe()
        self.imu = Probe()
        self.odom = Probe()
        self.joints = Probe()
        self.diag = Probe()
        self.cmd_out = Probe()
        self.description: str | None = None
        self.runaway = False
        # Set when the base controller has never reported telemetry. Everything
        # the bridge derives from it is then missing as a consequence, and
        # saying so once beats repeating it on every downstream check.
        self.base_mute = False

        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        # robot_state_publisher latches the description, so a late subscriber
        # only receives it with matching TRANSIENT_LOCAL durability.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)

        self.create_subscription(LaserScan, "/scan", self.scan.hit, sensor_qos)
        self.create_subscription(Imu, "/imu/data", self.imu.hit, sensor_qos)
        self.create_subscription(Odometry, "/odom", self.odom.hit, 20)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 20)
        self.create_subscription(DiagnosticArray, "/diagnostics", self.diag.hit, 10)
        self.create_subscription(Twist, "/cmd_vel_out", self.cmd_out.hit, 20)
        self.create_subscription(String, "/robot_description",
                                 self._on_description, latched)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _on_joints(self, msg: JointState) -> None:
        self.joints.hit(msg)
        if len(msg.velocity) >= 2:
            if max(abs(v) * RAD_S_TO_MM_S for v in msg.velocity[:2]) > RUNAWAY_MM_S:
                self.runaway = True

    def _on_description(self, msg: String) -> None:
        self.description = msg.data

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wheel_mm_s(self) -> list[float]:
        msg = self.joints.last
        if msg is None or len(msg.velocity) < 2:
            return [float("nan"), float("nan")]
        return [v * RAD_S_TO_MM_S for v in msg.velocity[:2]]

    def front_clearance(self) -> float:
        msg = self.scan.last
        if msg is None:
            return float("inf")
        nearest = float("inf")
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= math.radians(45) and math.isfinite(r) and r > 0.0:
                nearest = min(nearest, r)
        return nearest

    def stop(self, seconds: float = 0.8) -> None:
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            self.cmd_pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.02)

    def drive(self, linear: float, angular: float, seconds: float, settle: float):
        """Stream one command and average the wheel speeds and IMU yaw rate
        after 'settle'. Aborts the moment a wheel exceeds RUNAWAY_MM_S."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        wheels, yaw_rates, odom_vx = [], [], []
        started = time.monotonic()
        next_tx = started
        while rclpy.ok():
            now = time.monotonic()
            if now - started >= seconds:
                break
            if self.runaway:
                self.stop(1.2)
                return None, None, None
            if now >= next_tx:
                self.cmd_pub.publish(msg)
                next_tx += 1.0 / STREAM_HZ
            rclpy.spin_once(self, timeout_sec=0.02)
            if now - started >= settle:
                speeds = self.wheel_mm_s()
                if not any(math.isnan(v) for v in speeds):
                    wheels.append(speeds)
                if self.imu.last is not None:
                    yaw_rates.append(math.degrees(self.imu.last.angular_velocity.z))
                if self.odom.last is not None:
                    odom_vx.append(self.odom.last.twist.twist.linear.x * 1000.0)
        # Sampling has to finish before this: stop() publishes zeros and the
        # robot decelerates, so anything read afterwards describes a stopped
        # robot, not the one under test.
        self.stop()
        if not wheels:
            return None, None, None
        left = sum(w[0] for w in wheels) / len(wheels)
        right = sum(w[1] for w in wheels) / len(wheels)
        yaw = sum(yaw_rates) / len(yaw_rates) if yaw_rates else float("nan")
        vx = sum(odom_vx) / len(odom_vx) if odom_vx else float("nan")
        return (left, right), yaw, vx


# -----------------------------------------------------------------------------
# Phase 1: graph and base health
# -----------------------------------------------------------------------------

def phase_graph(node: Preflight, args) -> Phase:
    p = Phase("graph")

    node.spin(4.0)

    names = {n for n, _ in node.get_node_names_and_namespaces()}
    absent = []
    for required, why in (("mini_serial_bridge", "no Arduino link"),
                          ("coin_d6_lidar", "no LiDAR"),
                          ("mini_cmd_mux", "no velocity arbitration"),
                          ("robot_state_publisher", "no TF, no robot model")):
        if required in names:
            p.add(f"node {required}", PASS)
        else:
            absent.append(required)
            # Only the first missing node carries the fix: they all have the
            # same one, and four copies of it buries everything else.
            p.add(f"node {required}", FAIL, f"not running -- {why}",
                  "Start the robot stack on the Pi:\n"
                  "      ros2 launch mini_robot robot.launch.py"
                  if len(absent) == 1 else "")

    for topic, probe in (("/scan", node.scan), ("/imu/data", node.imu),
                         ("/odom", node.odom), ("/joint_states", node.joints),
                         ("/diagnostics", node.diag)):
        rate = probe.rate()
        floor = MIN_RATES[topic]
        if probe.count == 0:
            p.add(f"topic {topic}", FAIL, "no messages",
                  f"Nothing is publishing {topic}. Check the node that owns it.")
        elif rate < floor:
            p.add(f"topic {topic}", WARN, f"{rate:.1f} Hz (expected >= {floor:.0f})")
        else:
            p.add(f"topic {topic}", PASS, f"{rate:.1f} Hz")

    # The bridge's diagnostic is the only place the Arduino's own state reaches
    # ROS: a latched e-stop and a dead serial port both look like "the robot
    # ignores commands" from anywhere else.
    if node.diag.last is None:
        p.add("base health", FAIL, "no /diagnostics from the serial bridge")
        return p

    values = {}
    message = ""
    for status in node.diag.last.status:
        if "base controller" in status.name:
            message = status.message
            values = {kv.key: kv.value for kv in status.values}
    if not values:
        p.add("base health", WARN, "no base controller entry in /diagnostics")
        return p

    # Telemetry age is the load-bearing field. The bridge reports 'inf' when it
    # has never decoded a single 'T,' line, and in that state every other value
    # it publishes is a constructor default rather than a reading -- so they
    # must not be reported as if the robot had said them.
    age_text = values.get("telemetry_age_s", "inf")
    mute = age_text == "inf"
    last_rx = values.get("last_rx", "")

    if mute:
        # last_rx is whatever the board did say, and it usually names the fault
        # outright. A board answering ERR,UNKNOWN_COMMAND,VEL is alive, at the
        # right baud, and running a sketch that has no VEL command -- which
        # means the wrong sketch, not a wiring problem.
        if "UNKNOWN_COMMAND" in last_rx:
            detail = f"the board answers but does not know the command: '{last_rx}'"
            fix = ("The Arduino is running the WRONG SKETCH. It is alive and at the "
                   "right baud, but it has no VEL command and publishes no telemetry, "
                   "so the bridge has nothing to turn into /odom, /imu/data or TF.\n"
                   "      Flash arduino_mini_base_controller/ to the Uno and relaunch.\n"
                   "      The bench sketches (motor / IMU calibration) look like this.")
        elif last_rx:
            detail = f"no telemetry ever received; last line was '{last_rx}'"
            fix = ("The board is talking but never sends a 'T,' telemetry line. Check "
                   "which sketch is flashed and that the baud matches SERIAL_BAUD.")
        else:
            detail = "no telemetry ever received, and the board has said nothing at all"
            fix = ("The port opened but the Arduino is silent. Check it is powered and "
                   "actually flashed, that /dev/arduino_mini points at the Uno and not "
                   "the LiDAR, and that the baud matches SERIAL_BAUD in Config.h.")
        p.add("base telemetry", FAIL, detail, fix)
        # Say plainly that the rest of the base's numbers are meaningless here,
        # rather than printing defaults that read like healthy results.
        p.add("base state", SKIP,
              "e-stop / watchdog / IMU unknown: the base has not reported anything")
        node.base_mute = True
        # /imu/data, /odom and /joint_states are all decoded from the same
        # telemetry line. Their failures above are this one fault seen three
        # more times, so drop their generic advice rather than sending the
        # reader off to check three healthy nodes.
        for check in p.checks:
            if check.name in ("topic /imu/data", "topic /odom",
                              "topic /joint_states") and check.status == FAIL:
                check.detail += " -- decoded from base telemetry, see above"
                check.fix = ""
        return p

    p.add("base telemetry", PASS, f"{age_text} s old")
    p.add("base health", PASS if message.startswith("Base controller operational")
          else WARN, message)
    if values.get("estop_latched") == "True":
        p.add("e-stop", FAIL, "latched -- every velocity command is refused",
              "ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger {}")
    else:
        p.add("e-stop", PASS, "clear")
    if values.get("watchdog_stopped") == "True":
        p.add("command watchdog", WARN,
              "tripped -- nothing is streaming velocity yet (normal when idle)")
    else:
        p.add("command watchdog", PASS, "fed")
    if "imu" in values:
        p.add("IMU detected", PASS, values["imu"])
    return p


# -----------------------------------------------------------------------------
# Phase 2: do the sensors return physically sensible numbers
# -----------------------------------------------------------------------------

def phase_sensors(node: Preflight, args) -> Phase:
    p = Phase("sensors")

    if node.base_mute:
        p.add("base-derived sensors", SKIP,
              "IMU, odometry and joint states all come from base telemetry, "
              "which has not arrived -- fix that first")

    imu = node.imu.last
    if imu is None:
        p.add("IMU", SKIP if node.base_mute else FAIL,
              "no /imu/data" + (" (base is mute)" if node.base_mute else ""))
    else:
        magnitude = math.sqrt(imu.linear_acceleration.x ** 2 +
                              imu.linear_acceleration.y ** 2 +
                              imu.linear_acceleration.z ** 2)
        error = abs(magnitude - STANDARD_GRAVITY)
        detail = f"|accel| = {magnitude:.2f} m/s^2 at rest (expect ~9.81)"
        if error > 2.0:
            p.add("IMU accelerometer", FAIL, detail,
                  "The accelerometer is the filter's only gravity reference. This "
                  "far off means a wrong scale factor or a bad sensor; roll and "
                  "pitch cannot be trusted.")
        elif error > 0.7:
            p.add("IMU accelerometer", WARN, detail,
                  "Run the six-face calibration in arduino_mini_imu_calibration/.")
        else:
            p.add("IMU accelerometer", PASS, detail)

        drift = max(abs(imu.angular_velocity.x), abs(imu.angular_velocity.y),
                    abs(imu.angular_velocity.z))
        drift_deg = math.degrees(drift)
        if drift_deg > 2.0:
            p.add("IMU gyro bias", WARN, f"{drift_deg:.2f} deg/s while still",
                  "Yaw will drift. Keep the robot still and run: "
                  "ros2 service call /base/calibrate_imu std_srvs/srv/Trigger {}")
        else:
            p.add("IMU gyro bias", PASS, f"{drift_deg:.2f} deg/s while still")

        q = imu.orientation
        norm = math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2)
        p.add("IMU orientation", PASS if abs(norm - 1.0) < 0.05 else FAIL,
              f"quaternion norm {norm:.3f}")
        p.add("IMU frame", PASS if imu.header.frame_id == "imu_link" else WARN,
              f"frame_id '{imu.header.frame_id}' (expect imu_link)")

    scan = node.scan.last
    if scan is None:
        p.add("LiDAR", FAIL, "no /scan")
    else:
        total = len(scan.ranges)
        valid = sum(1 for r in scan.ranges
                    if math.isfinite(r) and scan.range_min <= r <= scan.range_max)
        fraction = valid / total if total else 0.0
        span = math.degrees(scan.angle_max - scan.angle_min)
        p.add("LiDAR geometry", PASS if span > 180.0 else WARN,
              f"{total} beams over {span:.0f} deg, "
              f"range {scan.range_min:.2f}..{scan.range_max:.1f} m")
        detail = f"{valid}/{total} beams valid ({100*fraction:.0f}%)"
        if fraction < 0.20:
            p.add("LiDAR returns", FAIL, detail,
                  "Almost nothing is coming back. Check the LiDAR is spinning, "
                  "unobstructed, and that it is not staring into open space.")
        elif fraction < 0.50:
            p.add("LiDAR returns", WARN, detail,
                  "Sparse. SLAM will struggle to match scans; map in a space with "
                  "walls inside range.")
        else:
            p.add("LiDAR returns", PASS, detail)
        p.add("LiDAR frame", PASS if scan.header.frame_id == "laser" else WARN,
              f"frame_id '{scan.header.frame_id}' (expect laser)")

    odom = node.odom.last
    if odom is None:
        p.add("odometry", SKIP if node.base_mute else FAIL,
              "no /odom" + (" (base is mute)" if node.base_mute else ""))
    else:
        ok = (odom.header.frame_id == "odom" and
              odom.child_frame_id == "base_footprint")
        p.add("odometry frames", PASS if ok else FAIL,
              f"{odom.header.frame_id} -> {odom.child_frame_id} "
              f"(expect odom -> base_footprint)")

    joints = node.joints.last
    if joints is None:
        p.add("joint states", SKIP if node.base_mute else FAIL,
              "no /joint_states" + (" (base is mute)" if node.base_mute else ""))
    else:
        missing = [j for j in DRIVEN_JOINTS if j not in list(joints.name)]
        if missing:
            p.add("joint states", FAIL, f"missing {', '.join(missing)}",
                  "RViz will not animate the wheels and the URDF joint names "
                  "disagree with the bridge's.")
        else:
            p.add("joint states", PASS, ", ".join(joints.name))
    return p


# -----------------------------------------------------------------------------
# Phase 3: the robot model and the TF tree
# -----------------------------------------------------------------------------

def phase_model(node: Preflight, args) -> Phase:
    p = Phase("model")

    if node.description is None:
        node.spin(2.0)
    if node.description is None:
        p.add("robot_description", FAIL, "not published",
              "robot_state_publisher is not up, or the xacro failed to expand. "
              "Check: ros2 launch mini_robot robot.launch.py")
        return p

    try:
        root = ET.fromstring(node.description)
    except ET.ParseError as exc:
        p.add("robot_description", FAIL, f"does not parse as XML: {exc}")
        return p

    links = {e.get("name") for e in root.findall("link")}
    joints = {e.get("name") for e in root.findall("joint")}
    p.add("robot_description", PASS,
          f"{len(links)} links, {len(joints)} joints, robot '{root.get('name')}'")

    for label, found, expected in (("links", links, EXPECTED_LINKS),
                                   ("joints", joints, EXPECTED_JOINTS)):
        missing = sorted(expected - found)
        if missing:
            p.add(f"URDF {label}", FAIL, f"missing {', '.join(missing)}",
                  "The description does not match what the rest of the stack "
                  "expects. RViz and SLAM both key off these names.")
        else:
            p.add(f"URDF {label}", PASS, f"all {len(expected)} present")

    # An unexpanded xacro substitution means the wheel macro never ran, which
    # produces a URDF that parses but describes a robot with no wheels.
    if any("${" in (name or "") for name in links | joints):
        p.add("xacro expansion", FAIL, "unexpanded ${...} in a link or joint name")
    else:
        p.add("xacro expansion", PASS, "no unexpanded substitutions")

    # TF is what SLAM actually consumes. A description that parses but never
    # reaches TF leaves scans unplaceable.
    transforms = [
        ("odom", "base_footprint", "published by the serial bridge"),
        ("base_footprint", "base_link", "static, from the URDF"),
        ("base_link", "laser", "static -- SLAM places every scan with this"),
        ("base_link", "imu_link", "static"),
    ]
    for parent, child, why in transforms:
        try:
            tf = node.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except Exception as exc:
            if parent == "odom" and node.base_mute:
                p.add(f"tf {parent}->{child}", SKIP,
                      "missing because the base reports no telemetry to build it from")
            else:
                p.add(f"tf {parent}->{child}", FAIL,
                      f"missing ({type(exc).__name__})",
                      f"{why}. Without it the TF tree is broken and SLAM cannot run.")
            continue
        t = tf.transform.translation
        p.add(f"tf {parent}->{child}", PASS,
              f"xyz {t.x:+.3f} {t.y:+.3f} {t.z:+.3f}")

    # The laser offset is the one number a wrong URDF gets silently wrong, and
    # every scan inherits the error.
    try:
        tf = node.tf_buffer.lookup_transform("base_link", "laser", rclpy.time.Time())
        t = tf.transform.translation
        if abs(t.x) < 1e-6 and abs(t.y) < 1e-6 and abs(t.z) < 1e-6:
            p.add("laser mounting", WARN, "laser sits exactly at base_link origin",
                  "Probably a placeholder. Measure the real offset and put it in "
                  "the laser_joint origin, or every scan is displaced by it.")
        else:
            p.add("laser mounting", PASS,
                  f"{t.z*1000:.0f} mm above base_link, "
                  f"{t.x*1000:+.0f} mm fore/aft")
    except Exception:
        pass
    return p


# -----------------------------------------------------------------------------
# Phase 4: does a command reach the wheels, and do they obey it
# -----------------------------------------------------------------------------

def phase_motion(node: Preflight, args) -> Phase:
    p = Phase("motion")

    if args.no_motion:
        p.add("motion", SKIP, "--no-motion given")
        return p

    # Check this before announcing anything: there is no point asking someone
    # to confirm the robot is on blocks for a test that cannot run.
    if node.base_mute:
        p.add("motion", SKIP,
              "the base reports no wheel telemetry, so driving it would prove "
              "nothing -- fix the base first")
        return p

    print()
    print("  The next checks SPIN THE WHEELS. The robot must be ON BLOCKS.")
    if not args.yes:
        try:
            if not input("  Ready? [y/N] ").strip().lower().startswith("y"):
                p.add("motion", SKIP, "declined")
                return p
        except EOFError:
            p.add("motion", SKIP, "no terminal to confirm on")
            return p

    clearance = node.front_clearance()
    if clearance < FRONT_STOP_M:
        p.add("front stop gate", WARN,
              f"obstacle at {clearance:.2f} m < {FRONT_STOP_M:.2f} m",
              "cmd_mux zeroes forward commands while this holds, which looks "
              "exactly like dead motors. Clear the space ahead of the LiDAR.")
    else:
        p.add("front stop gate", PASS, f"{clearance:.2f} m clear ahead")

    speed = args.speed
    turn = args.turn

    # Forward first: it is the direction every sign convention is defined from.
    wheels, _, odom_vx = node.drive(speed, 0.0, seconds=3.0, settle=1.5)
    if node.runaway:
        p.add("forward", FAIL, f"a wheel exceeded {RUNAWAY_MM_S:.0f} mm/s -- aborted",
              "Runaway wheel: its speed feedback carries the wrong sign, so the "
              "PID drives it harder instead of correcting. Fix the signs with "
              "scripts/validate_motor_control.py on the Pi before anything else.")
        return p
    if wheels is None:
        p.add("forward", FAIL, "no wheel telemetry while driving")
        return p

    target = speed * 1000.0
    left, right = wheels
    p.add("command reaches the wheels",
          PASS if max(abs(left), abs(right)) > 5.0 else FAIL,
          f"commanded {target:+.0f} mm/s -> L {left:+.0f}  R {right:+.0f} mm/s",
          "" if max(abs(left), abs(right)) > 5.0 else
          "Nothing moved. Check the mux is passing (/cmd_vel_out), the watchdog "
          "is fed, and the e-stop is clear.")

    for label, value in (("left", left), ("right", right)):
        if value < -5.0:
            p.add(f"{label} wheel direction", FAIL,
                  f"{value:+.0f} mm/s on a forward command",
                  "This wheel measures backwards. Run "
                  "scripts/validate_motor_control.py --tests link,signs on the Pi.")
        elif abs(value) < 5.0:
            p.add(f"{label} wheel direction", FAIL, "did not move")
        else:
            p.add(f"{label} wheel direction", PASS, f"{value:+.0f} mm/s forward")

    if abs(left) > 5.0 and abs(right) > 5.0:
        mismatch = abs(left - right)
        mean = 0.5 * (abs(left) + abs(right))
        detail = f"L {left:+.0f} vs R {right:+.0f} mm/s ({100*mismatch/mean:.0f}% apart)"
        if mismatch > 0.25 * mean:
            p.add("wheels matched", FAIL, detail,
                  "The robot will curve on a straight command and SLAM will fight "
                  "the odometry. Measure MEASURED_COUNTS_PER_REV per wheel and "
                  "re-run the feed-forward calibration.")
        elif mismatch > 0.10 * mean:
            p.add("wheels matched", WARN, detail)
        else:
            p.add("wheels matched", PASS, detail)

    # Odometry has to agree with what the wheels are doing, or the map drifts.
    if odom_vx is not None and not math.isnan(odom_vx):
        expected = 0.5 * (left + right)
        close = abs(odom_vx - expected) < max(15.0, 0.25 * abs(expected))
        p.add("odom twist", PASS if close else WARN,
              f"/odom reports {odom_vx:+.0f} mm/s, wheels average {expected:+.0f} mm/s",
              "" if close else
              "Odometry disagrees with the wheels it is derived from. SLAM will "
              "fight it. Check MEASURED_COUNTS_PER_REV and the wheel diameter.")

    node.spin(1.0)
    wheels, _, _ = node.drive(-speed, 0.0, seconds=2.5, settle=1.2)
    if wheels is not None and not node.runaway:
        left_r, right_r = wheels
        ok = left_r < -5.0 and right_r < -5.0
        p.add("reverse", PASS if ok else FAIL,
              f"L {left_r:+.0f}  R {right_r:+.0f} mm/s",
              "" if ok else "A wheel does not reverse. Check its H-bridge "
                            "direction pins.")

    # Rotation is where a sign error hides best: both wheels turn, the robot
    # spins the wrong way, and only the IMU disagrees.
    node.spin(1.0)
    wheels, yaw_rate, _ = node.drive(0.0, turn, seconds=3.0, settle=1.5)
    if wheels is not None and not node.runaway:
        left_t, right_t = wheels
        expected_deg = math.degrees(turn)
        p.add("rotate: wheel pattern",
              PASS if left_t < -2.0 and right_t > 2.0 else FAIL,
              f"commanded +{expected_deg:.0f} deg/s (CCW) -> "
              f"L {left_t:+.0f}  R {right_t:+.0f} mm/s",
              "" if left_t < -2.0 and right_t > 2.0 else
              "A CCW turn must drive the left wheel back and the right wheel "
              "forward. Swapped wheels would put SLAM's rotation backwards.")
        if not math.isnan(yaw_rate):
            if yaw_rate * expected_deg < 0:
                p.add("rotate: IMU agrees", FAIL,
                      f"commanded {expected_deg:+.0f} deg/s, IMU reads {yaw_rate:+.0f}",
                      "Yaw is inverted relative to the command. Flip IMU_YAW_SIGN "
                      "in Config.h -- REP-103 requires CCW positive, and SLAM will "
                      "map turns backwards until it is right.")
            else:
                p.add("rotate: IMU agrees", PASS,
                      f"commanded {expected_deg:+.0f} deg/s, IMU reads {yaw_rate:+.0f}")

    node.stop(1.0)
    return p


# -----------------------------------------------------------------------------
# Reporting
# -----------------------------------------------------------------------------

MARK = {PASS: "  ok  ", WARN: " warn ", FAIL: " FAIL ", SKIP: " skip "}


def report(phases: list[Phase]) -> int:
    print()
    print("=" * 74)
    print("PRE-FLIGHT SUMMARY")
    print("=" * 74)
    for phase in phases:
        print(f"\n  [{phase.status}] {phase.name}")
        for check in phase.checks:
            line = f"    {MARK[check.status]} {check.name}"
            if check.detail:
                line += f": {check.detail}"
            print(line)

    problems = [(ph.name, c) for ph in phases for c in ph.checks
                if c.status in (FAIL, WARN) and c.fix]
    if problems:
        print()
        print("-" * 74)
        print("WHAT TO FIX")
        print("-" * 74)
        for phase_name, check in problems:
            print(f"\n  [{phase_name}] {check.name}")
            print(f"    {check.fix}")

    failures = [c for ph in phases for c in ph.checks if c.status == FAIL]
    warnings = [c for ph in phases for c in ph.checks if c.status == WARN]
    print()
    print("=" * 74)
    if failures:
        print(f"NOT READY FOR SLAM -- {len(failures)} failure(s), "
              f"{len(warnings)} warning(s)")
        print("Fix the failures above, then re-run. Mapping on top of a broken")
        print("TF tree or a runaway wheel produces a map that cannot be salvaged.")
        return 2
    if warnings:
        print(f"READY FOR SLAM, with {len(warnings)} warning(s) worth reading first")
    else:
        print("READY FOR SLAM -- everything checked out")
    print()
    print("  On the workstation:")
    print("    ros2 launch mini_robot desktop_slam.launch.py")
    print("  Drive slowly and smoothly; the LiDAR sweeps at 10 Hz and scan")
    print("  matching is what turns wheel odometry into a map.")
    print("=" * 74)
    return 1 if warnings else 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check the mini robot is ready for a SLAM run.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--no-motion", action="store_true",
                        help="read-only: skip the phase that spins the wheels")
    parser.add_argument("--yes", action="store_true",
                        help="do not prompt before driving")
    parser.add_argument("--speed", type=float, default=0.08,
                        help="m/s used by the motion checks (default 0.08)")
    parser.add_argument("--turn", type=float, default=0.5,
                        help="rad/s used by the rotation check (default 0.5)")
    args = parser.parse_args()

    rclpy.init()
    node = Preflight()
    phases: list[Phase] = []
    try:
        print("mini pre-flight -- listening to the ROS graph...")
        for name, run in (("graph", phase_graph), ("sensors", phase_sensors),
                          ("model", phase_model), ("motion", phase_motion)):
            print(f"\n--- {name} " + "-" * (66 - len(name)))
            phase = run(node, args)
            for check in phase.checks:
                line = f"    {MARK[check.status]} {check.name}"
                if check.detail:
                    line += f": {check.detail}"
                print(line)
            phases.append(phase)
            # A dead graph makes every later phase report noise instead of
            # information, so stop rather than print it.
            if name == "graph" and any(
                    c.status == FAIL and c.name.startswith("node ")
                    for c in phase.checks):
                print("\n    Stopping: the stack is not running, so nothing below "
                      "would mean anything.")
                break
    except KeyboardInterrupt:
        print("\ninterrupted -- stopping the motors")
        node.stop(1.0)
    finally:
        try:
            node.stop(0.4)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    return report(phases)


if __name__ == "__main__":
    sys.exit(main())
