#!/usr/bin/env bash
# Source this file on BOTH the Raspberry Pi and Ubuntu workstation.
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=0

# BOTH MACHINES MUST USE THE SAME RMW. This is not a preference.
#
# ROS 2 topics are plain RTPS pub/sub and interoperate between Fast DDS and
# Cyclone DDS well enough that /scan, /odom and /tf all flow at full rate with
# the two mixed -- which is exactly what makes this so easy to miss. Services do
# not: request/reply needs matching correlation, so every call to /base/* from
# the workstation is discovered (wait_for_service returns true) and then times
# out. Measured on this robot: identical call, fastrtps replies, cyclonedds
# times out.
#
# Symptoms of getting this wrong are the RViz Reset panel doing nothing, and
# `ros2 service call` hanging while `ros2 topic echo` is perfectly happy.
#
# fastrtps, because it is the Humble default and the only RMW installed on the
# Pi. If you switch, install the same one on both and change it here.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
# For networks that block multicast, run a Fast DDS discovery server on the Pi
# and uncomment this on both machines:
# export ROS_DISCOVERY_SERVER=192.168.1.50:11811
