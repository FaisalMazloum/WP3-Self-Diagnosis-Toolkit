"""
Swarm controller with runtime pattern switching (publish "namespace:pattern" on /pattern_command).

Real robot (namespace = hostname, per system_config.yaml default):
    ros2 launch <ws>/ros2_launch/sw_system/remora_controller/swarm_controller_dynamic_switch.launch.py
Simulation, central mode for remora0..4:
    ros2 launch <ws>/ros2_launch/sw_system/remora_controller/swarm_controller_dynamic_switch.launch.py robot_id:=central robot_count:=5
"""
import os
import re
import socket

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# <ws>/ros2_launch/sw_system/remora_controller/
_WS_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', '..'))
DEFAULT_CONFIG_ROOT = os.path.join(_WS_ROOT, 'ros2_configs', 'sw_system', 'remora_controller')

# Launch-argument default meaning "use the value from system_config.yaml".
FROM_CONFIG = 'from_config'


def resolve_robot_id(robot_id):
    # 'central' (or '' in the YAML) -> central mode. A keyword is needed because ros2 launch rejects an empty value on the command line (robot_id:= is an error).
    if robot_id in ('', 'central'):
        return ''
    if robot_id != 'auto':
        return robot_id
    # ROS namespaces allow only letters, digits and underscores.
    return re.sub(r'[^A-Za-z0-9_]', '_', socket.gethostname())


def launch_setup(context, *args, **kwargs):
    config_root = os.path.abspath(os.path.expanduser(LaunchConfiguration('config_root').perform(context)))
    system_config_file = os.path.join(config_root, 'system_config.yaml')
    swarm_config_dir = os.path.join(config_root, 'swarm')
    pattern_dir = os.path.join(swarm_config_dir, 'movement_pattern', 'basic')
    hardware_protection_param_file = os.path.join(swarm_config_dir, 'hardware_protection_layer.yaml')

    with open(system_config_file) as f:
        system_config = yaml.safe_load(f) or {}

    def launch_setting(name):
        value = LaunchConfiguration(name).perform(context)
        if value != FROM_CONFIG:
            return value
        if name not in system_config:
            raise RuntimeError(f'"{name}" missing from {system_config_file}')
        return system_config[name]

    robot_id = resolve_robot_id(str(launch_setting('robot_id') or ''))
    robot_count = int(launch_setting('robot_count'))
    initial_pattern = str(launch_setting('initial_pattern'))

    initial_pattern_file = os.path.join(pattern_dir, f'{initial_pattern}.yaml')
    if not os.path.isfile(initial_pattern_file):
        raise RuntimeError(f'initial_pattern "{initial_pattern}" has no config file {initial_pattern_file}')

    # robot_id set -> per-robot mode: this launch runs on one robot and manages only it (robot_count is ignored).
    # Empty -> central mode for remora0..robot_count-1.
    namespaces = [robot_id] if robot_id else [f'remora{i}' for i in range(robot_count)]
    shared_node_namespace = robot_id if robot_id else None

    supervisor_params = {
        'robot_count': robot_count,
        'robot_id': robot_id,
        'initial_pattern': initial_pattern,
    }

    actions = []

    for namespace in namespaces:
        # Hardware protection node stays up the whole time, unaffected by pattern switching.
        actions.append(Node(
            package='ros2swarm',
            executable='hardware_protection_layer',
            namespace=namespace,
            parameters=[hardware_protection_param_file],
            remappings=[
                ('range_data', 'gnss_range_data')
            ],
            output='screen'
        ))

    # Reports each robot's current pattern on /remoraN/current_pattern for the simulator UI.
    actions.append(Node(
        package='remora_controller',
        executable='pattern_status_publisher',
        namespace=shared_node_namespace,
        parameters=[supervisor_params],
        output='screen'
    ))

    # Owns the actual ros2swarm pattern process per robot; switches on /pattern_command.
    actions.append(Node(
        package='remora_controller',
        executable='pattern_supervisor',
        namespace=shared_node_namespace,
        parameters=[{**supervisor_params, 'swarm_movement_pattern_dir': pattern_dir}], # copy everything from supervisor_params + add swarm_movement_pattern_dir kvp 
        output='screen'
    ))

    # GNSS-based virtual range sensor for the same robot(s).
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_WS_ROOT, 'ros2_launch', 'sw_system', 'remora_controller', 'gnss_range_sensor.launch.py')
        ),
        launch_arguments={
            'config_root': config_root,
            'robot_id': robot_id if robot_id else 'central',
            'robot_count': str(robot_count),
        }.items()
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_root',
            default_value=DEFAULT_CONFIG_ROOT,
            description='Folder holding system_config.yaml and swarm/ configs',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value=FROM_CONFIG,
            description="'auto' = hostname, 'central' = central mode, or a namespace like remora3. "
                        'Default: system_config.yaml',
        ),
        DeclareLaunchArgument(
            'robot_count',
            default_value=FROM_CONFIG,
            description='Robots in central mode (ignored when robot_id is set). Default: system_config.yaml',
        ),
        DeclareLaunchArgument(
            'initial_pattern',
            default_value=FROM_CONFIG,
            description='Pattern each robot starts on. Default: system_config.yaml',
        ),
        OpaqueFunction(function=launch_setup),
    ])
