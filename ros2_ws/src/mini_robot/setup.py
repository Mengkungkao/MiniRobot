from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'mini_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    # Install launch files, YAML configs, and URDF into the package's share
    # directory so launch files and get_package_share_directory() can find them.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools', 'pyserial', 'PyYAML'],
    zip_safe=True,
    maintainer='Meng Kung Kao',
    maintainer_email='77830467+Mengkungkao@users.noreply.github.com',
    description='mini ROS2 two-wheel differential drive robot stack',
    license='MIT',
    # Executables exposed to `ros2 run mini_robot <name>`.
    entry_points={
        'console_scripts': [
            'serial_bridge = mini_robot.serial_bridge:main',
            'coin_d6_lidar = mini_robot.coin_d6_lidar:main',
            'cmd_mux = mini_robot.cmd_mux:main',
            'waypoint_cli = mini_robot.waypoint_cli:main',
            'goto_cli = mini_robot.goto_cli:main',
            'teleop_keyboard = mini_robot.teleop_keyboard:main',
            'nav_monitor = mini_robot.nav_monitor:main',
        ],
    },
)
