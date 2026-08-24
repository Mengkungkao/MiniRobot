# On-robot bringup for the Raspberry Pi 4B: robot_state_publisher (URDF/TF),
# the Arduino serial bridge, the COIN-D6 LiDAR driver, and the velocity mux.
# SLAM/Nav2/RViz run separately on the desktop (see desktop_*.launch.py).
#
# Every node here is optional and can be left out without breaking the rest:
#
#   ros2 launch mini_robot robot.launch.py                  everything
#   ros2 launch mini_robot robot.launch.py lidar:=false     no LiDAR: the robot
#       still drives on teleop and dead reckoning; the mux's front stop gate
#       has nothing to gate on and says so (require_scan in base.yaml decides
#       whether it then blocks forward motion or passes it through). Mapping
#       and Nav2 need scans, so those stop working.
#   ros2 launch mini_robot robot.launch.py mux:=false       no arbitration and
#       no stop gate: the bridge subscribes to /cmd_vel directly, so Nav2 and
#       anything else publishing there drive the base with nothing in between.
#   ros2 launch mini_robot robot.launch.py base:=false      no Arduino: LiDAR
#       and TF only, for working on perception with the base powered down.
#   ros2 launch mini_robot robot.launch.py description:=false   no URDF/TF, for
#       when something else publishes the robot description.
#
# The switches compose, and a node that is off is simply never started -- there
# is no stub, no zero-publisher and nothing pretending the hardware is there.
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Installed config files and the xacro-expanded robot description.
    share = get_package_share_directory('mini_robot')
    base = os.path.join(share, 'config', 'base.yaml')
    lidar = os.path.join(share, 'config', 'lidar.yaml')
    urdf = os.path.join(share, 'urdf', 'mini_robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf]), value_type=str)

    use_mux = LaunchConfiguration('mux')

    # With the mux running, the bridge consumes its arbitrated output. Without
    # it, nothing would ever publish /cmd_vel_out and the base would sit deaf
    # while commands piled up on /cmd_vel, so point it at /cmd_vel instead.
    # This overrides cmd_vel_topic from base.yaml.
    bridge_cmd_vel_topic = PythonExpression(
        ["'/cmd_vel_out' if '", use_mux, "'.lower() in ('true', '1') else '/cmd_vel'"])

    return LaunchDescription([
        # Device ports are overridable arguments; the defaults are udev
        # symlinks so the nodes do not depend on USB enumeration order.
        DeclareLaunchArgument('arduino_port', default_value='/dev/arduino_mini'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/coin_d6'),
        DeclareLaunchArgument('description', default_value='true',
                              description='publish the URDF and its TF tree'),
        DeclareLaunchArgument('base', default_value='true',
                              description='run the Arduino serial bridge'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='run the COIN-D6 LiDAR driver'),
        DeclareLaunchArgument('mux', default_value='true',
                              description='run the velocity mux and front stop gate; '
                                          'when false the base bridge reads /cmd_vel directly'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             condition=IfCondition(LaunchConfiguration('description')),
             parameters=[{'robot_description': robot_description, 'use_sim_time': False}], output='screen'),
        Node(package='mini_robot', executable='serial_bridge', name='mini_serial_bridge',
             condition=IfCondition(LaunchConfiguration('base')),
             parameters=[base, {'port': LaunchConfiguration('arduino_port'),
                                'cmd_vel_topic': bridge_cmd_vel_topic}], output='screen'),
        Node(package='mini_robot', executable='coin_d6_lidar', name='coin_d6_lidar',
             condition=IfCondition(LaunchConfiguration('lidar')),
             parameters=[lidar, {'port': LaunchConfiguration('lidar_port')}], output='screen'),
        Node(package='mini_robot', executable='cmd_mux', name='mini_cmd_mux',
             condition=IfCondition(use_mux),
             parameters=[base], output='screen'),
    ])
