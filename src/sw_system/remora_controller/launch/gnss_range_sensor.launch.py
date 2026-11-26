"""
Starts gnss_range_sensor for one robot or for remora0 .. remora{N-1}.

Parameters come from <config_root>/gnss_range_sensor.yaml. Any parameter can be overridden
on the command line; only the ones you type replace the YAML values.

Real robot (namespace = hostname):
    ros2 launch remora_controller gnss_range_sensor.launch.py robot_id:=auto

Simulation, remora0..4, Unity IMU:
    ros2 launch remora_controller gnss_range_sensor.launch.py robot_count:=5 imu_yaw_offset_deg:=90
"""
import os
import re
import socket

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Package copy of the configs (the reference template).
DEFAULT_CONFIG_ROOT = os.path.join(get_package_share_directory('remora_controller'), 'config')

# Launch-argument default meaning "use the value from gnss_range_sensor.yaml".
FROM_CONFIG = 'from_config'

# Node parameters that can be overridden from the command line (all must exist in the YAML).
# The node's own robot_id parameter is not listed: the node takes its id from its namespace.
OVERRIDABLE_PARAMS = [
    'status_topic',
    'output_topic',
    'angle_min_deg',
    'angle_max_deg',
    'angle_increment_deg',
    'range_max',
    'robot_radius',
    'scan_rate',
    'status_rate',
    'stale_timeout',
    'use_yaw',
    'angle_direction',
    'imu_yaw_offset_deg',
]


def resolve_robot_id(robot_id):
    # Same meaning as in swarm_controller_dynamic_switch.launch.py.
    if robot_id in ('', 'central'):
        return ''
    if robot_id != 'auto':
        return robot_id
    return re.sub(r'[^A-Za-z0-9_]', '_', socket.gethostname())


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    config_root = os.path.abspath(os.path.expanduser(arg('config_root')))
    params_file = os.path.join(config_root, 'gnss_range_sensor.yaml')

    with open(params_file) as f:
        yaml_params = (yaml.safe_load(f) or {}).get('/**', {}).get('ros__parameters', {})

    # Only the arguments that were typed go into overrides; everything else stays as in the YAML.
    overrides = {}
    for name in OVERRIDABLE_PARAMS:
        value = arg(name)
        if value == FROM_CONFIG:
            continue
        typed = yaml.safe_load(value)   # '10.0' -> 10.0, 'true' -> True, 'range_data' -> 'range_data'
        if isinstance(yaml_params.get(name), float) and isinstance(typed, int) and not isinstance(typed, bool):
            typed = float(typed)
        overrides[name] = typed

    robot_id = resolve_robot_id(arg('robot_id'))
    namespaces = [robot_id] if robot_id else [f'remora{i}' for i in range(int(arg('robot_count')))]

    return [
        Node(
            package='remora_controller',
            executable='gnss_range_sensor',
            namespace=ns,
            parameters=[params_file, overrides],   # later entries win
            output='screen',
        )
        for ns in namespaces
    ]


def generate_launch_description():
    param_args = [
        DeclareLaunchArgument(name, default_value=FROM_CONFIG,
                              description='Overrides gnss_range_sensor.yaml. Default: value in the YAML')
        for name in OVERRIDABLE_PARAMS
    ]
    return LaunchDescription([
        DeclareLaunchArgument('config_root', default_value=DEFAULT_CONFIG_ROOT,
                              description='Folder holding gnss_range_sensor.yaml'),
        DeclareLaunchArgument('robot_id', default_value='',
                              description="'auto' = hostname, a namespace like remora3, or ''/'central' = "
                                          'remora0..robot_count-1'),
        DeclareLaunchArgument('robot_count', default_value='1',
                              description='Number of robots when robot_id is empty/central'),
        *param_args,
        OpaqueFunction(function=launch_setup),
    ])
