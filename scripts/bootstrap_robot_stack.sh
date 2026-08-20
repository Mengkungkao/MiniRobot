#!/usr/bin/env bash
set -euo pipefail

# One-shot installer for the mini robot stack. Safe to re-run.
#
#   bash scripts/bootstrap_robot_stack.sh pi        on the Raspberry Pi 4B
#   bash scripts/bootstrap_robot_stack.sh desktop   on the Ubuntu workstation

MODE="${1:-desktop}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
WORKSPACE_DIR="${WORKSPACE_DIR:-$HOME/ros2_ws}"
ROS_DOMAIN="${ROS_DOMAIN_ID:-31}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

# Packages to symlink into the colcon workspace. mini_rviz_plugins is a Qt/RViz
# C++ package: it only builds where rviz_common and the Qt5 dev files exist, so
# the Pi (ros-base, headless) gets the Python package only. Adding it there
# would fail the whole `colcon build`, taking the robot-side nodes down with it.
WORKSPACE_PACKAGES=(mini_robot)

case "$MODE" in
  desktop)
    echo "Installing ROS 2 desktop stack for the Ubuntu workstation"
    INSTALL_PACKAGES=(
      "ros-${ROS_DISTRO}-desktop"
      "ros-${ROS_DISTRO}-navigation2"
      "ros-${ROS_DISTRO}-nav2-bringup"
      "ros-${ROS_DISTRO}-slam-toolbox"
      "ros-${ROS_DISTRO}-teleop-twist-keyboard"
      "qtbase5-dev"
      "python3-colcon-common-extensions"
      "python3-yaml"
      "python3-serial"
      "chrony"
    )
    WORKSPACE_PACKAGES+=(mini_rviz_plugins)
    ;;
  pi)
    echo "Installing ROS 2 base stack for the Raspberry Pi"
    INSTALL_PACKAGES=(
      "ros-${ROS_DISTRO}-ros-base"
      "ros-${ROS_DISTRO}-robot-state-publisher"
      "ros-${ROS_DISTRO}-xacro"
      "python3-colcon-common-extensions"
      "python3-yaml"
      "python3-serial"
      "chrony"
    )
    ;;
  *)
    echo "Usage: $0 [desktop|pi]" >&2
    exit 1
    ;;
esac

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required" >&2
  exit 1
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "This installer expects Ubuntu/Debian apt" >&2
  exit 1
fi

# apt refuses two different Signed-By values for the same repo, so a legacy
# hand-rolled ros2.list alongside the official deb822 ros2.sources breaks
# `apt update` outright. Remove the legacy file before installing the official
# source below.
if [ -f /etc/apt/sources.list.d/ros2.list ] && grep -q "packages.ros.org" /etc/apt/sources.list.d/ros2.list 2>/dev/null; then
  echo "Removing legacy /etc/apt/sources.list.d/ros2.list (superseded by the official ros2-apt-source package)"
  sudo rm -f /etc/apt/sources.list.d/ros2.list
  sudo rm -f /usr/share/keyrings/ros-archive-keyring.gpg
fi

OTHER_ROS_SOURCES="$(grep -rl "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/ 2>/dev/null | grep -vx "/etc/apt/sources.list.d/ros2.sources" || true)"
if [ -n "$OTHER_ROS_SOURCES" ]; then
  echo "Found packages.ros.org source(s) this script does not manage:" >&2
  echo "$OTHER_ROS_SOURCES" >&2
  echo "Remove the duplicate(s) to avoid a Signed-By conflict, then re-run." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y curl ca-certificates

if [ ! -f /etc/apt/sources.list.d/ros2.sources ]; then
  ROS_CODENAME="$(. /etc/os-release && echo "$UBUNTU_CODENAME")"
  # Buffer the API response before grepping: grep -m1 closing the pipe early
  # makes curl exit 23 under pipefail.
  ROS_APT_RELEASE_JSON="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest)"
  ROS_APT_SOURCE_VERSION="$(printf '%s\n' "$ROS_APT_RELEASE_JSON" | grep -m1 '"tag_name"' | cut -d'"' -f4)"
  if [ -z "$ROS_APT_SOURCE_VERSION" ]; then
    echo "Could not determine the current ros2-apt-source release from GitHub" >&2
    exit 1
  fi
  ROS_APT_DEB="$(mktemp --suffix=.deb)"
  curl -fL -o "$ROS_APT_DEB" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${ROS_CODENAME}_all.deb"
  sudo dpkg -i "$ROS_APT_DEB"
  rm -f "$ROS_APT_DEB"
fi

sudo apt-get update
sudo apt-get install -y "${INSTALL_PACKAGES[@]}" python3-rosdep python3-rosinstall-generator python3-vcstool build-essential

sudo rosdep init >/dev/null 2>&1 || true
rosdep update >/dev/null 2>&1 || true

# Serial port access for the Arduino and the LiDAR.
sudo usermod -aG dialout "$USER" >/dev/null 2>&1 || true

mkdir -p "$WORKSPACE_DIR/src"
for pkg in "${WORKSPACE_PACKAGES[@]}"; do
  TARGET="$WORKSPACE_DIR/src/$pkg"
  if [ -e "$TARGET" ] && [ ! -L "$TARGET" ]; then
    # A real (non-symlink) copy here silently shadows this repo: colcon builds
    # and installs whatever stale code happens to be in that copy, `git pull`
    # never touches it, and `ros2 launch` errors about missing files that
    # clearly exist in the repo. Move it aside instead of leaving that
    # mismatch to be debugged by hand.
    STALE_BACKUP="$TARGET.stale-backup.$(date +%Y%m%d%H%M%S)"
    echo "WARNING: $TARGET is a real directory, not a symlink back to this repo," >&2
    echo "so it was going stale. Moving it to $STALE_BACKUP and linking the repo." >&2
    mv "$TARGET" "$STALE_BACKUP"
    ln -s "$REPO_ROOT/ros2_ws/src/$pkg" "$TARGET"
  elif [ ! -e "$TARGET" ]; then
    ln -s "$REPO_ROOT/ros2_ws/src/$pkg" "$TARGET"
  fi
done

# On the Pi, drop a stale mini_rviz_plugins link from a previous desktop-mode
# run: ros-base has no rviz_common, so leaving it would fail every build.
if [ "$MODE" = "pi" ] && [ -L "$WORKSPACE_DIR/src/mini_rviz_plugins" ]; then
  echo "Removing $WORKSPACE_DIR/src/mini_rviz_plugins (needs rviz_common, not available on ros-base)"
  rm -f "$WORKSPACE_DIR/src/mini_rviz_plugins"
fi

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  cd "$WORKSPACE_DIR"
  rosdep install --from-paths src --ignore-src -r -y || true
  colcon build --symlink-install
fi

if [ "$MODE" = "pi" ]; then
  GENERATED_RULES="$REPO_ROOT/scripts/99-mini-robot.rules"

  # Prefer auto-detected rules. detect_serial_ports.sh identifies each adapter
  # by what it transmits, which is the only thing that separates two CH340s.
  # Its output is per-machine, so it is generated here rather than tracked.
  if [ ! -f "$GENERATED_RULES" ] && ls /dev/ttyUSB* /dev/ttyACM* >/dev/null 2>&1; then
    echo "Detecting which serial adapter is the Arduino and which is the LiDAR..."
    bash "$REPO_ROOT/scripts/detect_serial_ports.sh" \
      || echo "Detection failed; falling back to the checked-in rules file." >&2
  fi

  RULES_SRC=""
  if [ -f "$GENERATED_RULES" ]; then
    RULES_SRC="$GENERATED_RULES"
  elif ! grep -q "_HERE" "$REPO_ROOT/scripts/99-mini-robot.rules.example"; then
    RULES_SRC="$REPO_ROOT/scripts/99-mini-robot.rules.example"
  fi

  if [ -n "$RULES_SRC" ]; then
    sudo install -m 0644 "$RULES_SRC" /etc/udev/rules.d/99-mini-robot.rules
    sudo udevadm control --reload-rules >/dev/null 2>&1 || true
    sudo udevadm trigger --subsystem-match=tty >/dev/null 2>&1 || true
    echo "Installed udev rules from $(basename "$RULES_SRC")"
  else
    echo "Skipping udev rule install: no usable rules file yet."
    echo "  Plug in both the Arduino and the LiDAR, then run:"
    echo "    bash scripts/detect_serial_ports.sh --install"
  fi
fi

BASHRC_SNIPPET="""
# mini ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN}
export ROS_LOCALHOST_ONLY=0
"""

if ! grep -Fq "mini ROS 2 environment" "$HOME/.bashrc" 2>/dev/null; then
  printf '\n%s\n' "$BASHRC_SNIPPET" >> "$HOME/.bashrc"
fi

echo "Bootstrap completed. Open a new shell or run:"
echo "  source /opt/ros/${ROS_DISTRO}/setup.bash"
echo "  source ${WORKSPACE_DIR}/install/setup.bash"
