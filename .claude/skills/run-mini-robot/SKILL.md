---
name: run-mini-robot
description: Launch this two-wheel differential drive robot stack and verify it came up — desk mode (URDF/TF and RViz, no hardware), the desktop SLAM/Nav2 session, or full on-robot bringup on the Raspberry Pi. Use when asked to run, start, launch, or screenshot the mini robot, or to confirm a change works in the running system.
---

# Running mini

ROS 2 Humble. Split stack: the Raspberry Pi 4B runs the hardware nodes, an
Ubuntu workstation runs SLAM/Nav2/RViz, and DDS carries topics between them.
**There is no simulator** — anything below that touches the base drives real
motors.

## Environment — read this first

The packages live in `~/MiniRobot/ros2_ws/src/`, but that workspace is **not**
where they get built. They are symlinked into `~/ros2_ws`, which is the build
workspace and the one that has an `install/`:

```
~/ros2_ws/src/mini_robot        -> ~/MiniRobot/ros2_ws/src/mini_robot
~/ros2_ws/src/mini_rviz_plugins -> ~/MiniRobot/ros2_ws/src/mini_rviz_plugins
```

Edit under `~/MiniRobot`, build in `~/ros2_ws`. Do not create a second
`install/` under `~/MiniRobot/ros2_ws` — two installs of the same packages on
one machine is how you end up debugging a stale copy.

`~/.bashrc` already defines a `minirobot` shell function that sets all of this
up. Because it is a function, it does not exist in a non-interactive shell, so
inline it:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

`local_setup.bash`, not `setup.bash`: a colcon `setup.bash` replays the underlay
chain recorded at build time, which is how mdetect's and the arm's packages used
to end up in a MiniRobot shell. `local_setup.bash` adds only this workspace.

All three exports matter. Domain 31 is this project's; mdetect uses 30 and they
must not be mixed. The RMW must be **fastrtps** — it has to match the Pi, which
has only fastrtps. Topics interoperate between fastrtps and cyclonedds but
services do not, so a cyclonedds shell gives you a stack that looks alive and
silently fails every service call.

## Build

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select mini_robot mini_rviz_plugins
source install/setup.bash
```

`--packages-select` matters here: `~/ros2_ws/src` also holds `cspc_lidar_sdk_ros2`,
`ros_tutorials` and `examples`, and a bare `colcon build` rebuilds all of them.
This workspace is MiniRobot's alone — mdetect used to be symlinked in here and
now builds in its own repo instead.

`mini_rviz_plugins` is C++ (the Goto and Teleop panels) and must be rebuilt
after changes; `mini_robot` is Python and, with `--symlink-install`, does not.

## Pick a mode

### desk — default for agent-driven runs

Description and TF only. No serial port, no LiDAR, **no motors**. This is what
to run to check that a URDF/xacro change parses and the TF tree is sane, and it
is safe on a workstation with no robot attached.

```bash
ros2 launch mini_robot robot.launch.py base:=false lidar:=false mux:=false
```

Then, for a picture, plain `rviz2` with Fixed Frame `base_footprint`. Add
`joint_state_publisher_gui` if joints need moving by hand. The bundled
`mini_view.rviz` config is a Nav2 view and will show errors without a map, so
prefer a bare RViz here.

### desktop — the SLAM or Nav2 session

Runs on the workstation and **requires a live robot** publishing `/scan` and
`/odom` on domain 31. Without one, slam_toolbox and Nav2 start and then sit
there waiting; that is not a failure of your change.

```bash
ros2 launch mini_robot desktop_slam.launch.py         # mapping
ros2 launch mini_robot desktop_navigation.launch.py   # navigating a saved map
```

`python3 scripts/preflight_slam.py` checks the prerequisites before you start.

### robot — full on-robot bringup

**Never start this on your own initiative.** It opens the Arduino serial port
and the wheels can turn immediately. Run it only when the user has asked for
the real robot in this session, on the Pi, and say what will move first.

```bash
ros2 launch mini_robot robot.launch.py
```

Every node is individually switchable — `description:`, `base:`, `lidar:`,
`mux:` — and a node that is off is simply never started; there is no stub
pretending the hardware is there. Header comments in the launch file spell out
what each subset costs you.

`scripts/robot_bringup.sh` is the systemd entry point, not a hand-run command.
It stands aside (exit 0) if it sees a hand-started `serial_bridge` or
`coin_d6_lidar`, so a manual launch always wins.

## Running alongside the other projects

This machine hosts three ROS 2 projects that must not see each other. `~/.bashrc`
already documents why the choice is explicit rather than sourcing-order
dependent: the three env blocks used to fight, the last `ROS_DOMAIN_ID` won, and
mdetect silently ran on MiniRobot's domain while the RMW was pinned to
cyclonedds so MiniRobot could not call services on the Pi. Neither project got
what it asked for and nothing reported it.

| Project | Workspace sourced | Domain | RMW |
|---|---|---|---|
| MiniRobot (this one) | `~/ros2_ws/install` | **31** | **fastrtps** (must match the Pi) |
| mdetect | `~/mdetect_ws/install` | 30 | cyclonedds |
| robot_arm | `~/RobotArm/robot_arm_ws/install` | 32 | fastrtps |

One project per shell. MiniRobot is the one project here whose RMW is not a
preference — the Pi has only fastrtps installed, so a cyclonedds shell on this
end gives topics that flow and services that never answer.

Note `ROS_LOCALHOST_ONLY=0` for MiniRobot, unlike the arm: the Pi is a separate
machine and localhost-only would cut it off entirely.

With distinct domains all three can run at once on this workstation. Check
before blaming a change:

```bash
echo "domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION"
ros2 node list          # only mini_* / nav2 / slam_toolbox nodes should appear
```

A `robot_arm_*` or `mdetect_*` node in that list means the domains have collided.

## Verify it actually came up

```bash
ros2 node list
ros2 topic list
ros2 run tf2_tools view_frames        # writes frames_*.pdf
```

- **desk**: `/robot_state_publisher` present, `/tf_static` populated, TF chain
  `base_footprint -> base_link -> laser`.
- **desktop mapping**: add `map -> odom` from slam_toolbox, and
  `ros2 topic hz /scan` non-zero.
- **robot**: `ros2 topic hz /odom /imu/data /joint_states` all non-zero.

Watch which `/cmd_vel*` topic is live — the arbitration is the easiest thing to
get wrong. Nav2 always publishes on plain `/cmd_vel` (its internal remap is
hardcoded), `cmd_mux` reads that plus `/cmd_vel_teleop` and `/cmd_vel_manual`,
and republishes the gated result on **`/cmd_vel_out`**, which is the only topic
the serial bridge listens to. With `mux:=false` the bridge falls back to
`/cmd_vel` directly, with nothing in between — including no front stop gate.

## Drive it by hand

```bash
ros2 run mini_robot teleop_keyboard     # publishes /cmd_vel_teleop
ros2 run mini_robot goto_cli            # x_mm,y_mm,heading_deg
ros2 run mini_robot nav_monitor
```

Convention: `+X` forward, `+Y` left, positive yaw counter-clockwise.

## Shut down

Ctrl-C the launch, then confirm the ports were released — a surviving node
holds the Arduino open and the next run fails with a port-busy error that
looks like a hardware fault:

```bash
pgrep -af 'serial_bridge|coin_d6_lidar|cmd_mux|slam_toolbox' || echo clean
```

## When it fails

- **`package 'mini_robot' not found`** — sourced `~/MiniRobot/ros2_ws/install`
  (which does not exist) instead of `~/ros2_ws/install`.
- **Nodes visible but services time out** — RMW mismatch with the Pi. Both ends
  must be `rmw_fastrtps_cpp`.
- **Nothing visible at all** — `ROS_DOMAIN_ID` is not 31, or
  `ROS_LOCALHOST_ONLY=1`.
- **`/dev/arduino_mini` or `/dev/coin_d6` missing** — udev symlinks absent.
  `bash scripts/detect_serial_ports.sh --install`. The launch takes
  `arduino_port:=` / `lidar_port:=` overrides for a one-off.
- **Robot will not drive forward** — the mux's front stop gate. With no LiDAR,
  `require_scan` in `config/base.yaml` decides whether it blocks or passes.
