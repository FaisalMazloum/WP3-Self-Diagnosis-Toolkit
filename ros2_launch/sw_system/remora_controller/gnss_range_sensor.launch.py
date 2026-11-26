"""
Workspace version of remora_controller's gnss_range_sensor.launch.py.

Runs the package launch with configs from <ws>/ros2_configs/sw_system/remora_controller.
All arguments (robot_id, robot_count, parameter overrides) pass through, e.g.
    ros2 launch <ws>/ros2_launch/sw_system/remora_controller/gnss_range_sensor.launch.py robot_id:=auto
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# This file lives in <ws>/ros2_launch/sw_system/remora_controller/, so <ws> is three levels up.
_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))
CONFIG_ROOT = os.path.join(_WS_ROOT, 'ros2_configs', 'sw_system', 'remora_controller')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('remora_controller'), 'launch', 'gnss_range_sensor.launch.py')),
            launch_arguments={'config_root': CONFIG_ROOT}.items(),
        ),
    ])
