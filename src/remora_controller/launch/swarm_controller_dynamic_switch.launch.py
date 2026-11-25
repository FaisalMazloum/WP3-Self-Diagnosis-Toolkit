from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

PATTERN_CHOICES = [
    'drive_pattern', 'random_walk_pattern', 'dispersion_pattern',
    'aggregation_pattern', 'attraction_pattern', 'attraction_pattern2',
    'minimalist_flocking_pattern', 'adaptive_sampling_pattern'
]


def launch_setup(context, *args, **kwargs):
    initial_pattern = LaunchConfiguration('initial_pattern').perform(context)
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    robot_id = LaunchConfiguration('robot_id').perform(context)

    # robot_id set -> per-robot mode: this launch runs on one robot and manages only it (robot_count is ignored). 
    # Empty -> central mode for remora0..robot_count-1.
    namespaces = [robot_id] if robot_id else [f'remora{i}' for i in range(robot_count)]
    # In per-robot mode, put the shared nodes under the robot's namespace so N robots don't
    # all run nodes named /pattern_supervisor. Their topics are absolute, so they are unaffected.
    shared_node_namespace = robot_id if robot_id else None

    hardware_protection_param_file = PathJoinSubstitution([
        FindPackageShare('remora_controller'),
        'config',
        'swarm',
        'burger',
        'hardware_protection_layer.yaml'
    ])


    actions = []

    for namespace in namespaces:
        # Hardware protection node stays up the whole time, unaffected by pattern switching.
        actions.append(Node(
            package="ros2swarm",
            executable='hardware_protection_layer',
            namespace=namespace,
            parameters=[hardware_protection_param_file],
            remappings=[
                # ('range_data', 'obstacle_ranges')
                ('range_data', 'gnss_range_data')
            ],
            output="screen"
        ))

    # Reports each robot's current pattern on /remoraN/current_pattern for the simulator UI.
    actions.append(Node(
        package="remora_controller",
        executable='pattern_status_publisher',
        namespace=shared_node_namespace,
        parameters=[{
            'robot_count': robot_count,
            'robot_id': robot_id,
            'initial_pattern': initial_pattern,
        }],
        output="screen"
    ))

    # Owns the actual ros2swarm pattern process per robot; switches on /pattern_command.
    actions.append(Node(
        package="remora_controller",
        executable='pattern_supervisor',
        namespace=shared_node_namespace,
        parameters=[{
            'robot_count': robot_count,
            'robot_id': robot_id,
            'initial_pattern': initial_pattern,
        }],
        output="screen"
    ))

    # Runs the GNSS range_data nodes
    actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('remora_controller'),
                    'launch',
                    'gnss_range_sensor.launch.py'
                )
            ),
            launch_arguments={
                # 'robot_id': robot_id,
                'robot_id': 'remora0',
                # 'output_topic': 'range_data'
            }.items()
        ))

    return actions


def generate_launch_description():
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots in the swarm (central mode only; ignored when robot_id is set)',
    )

    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='',
        description='Run in per-robot mode for just this robot (e.g. remora7). Empty = central mode.',
    )

    initial_pattern_arg = DeclareLaunchArgument(
        'initial_pattern',
        default_value='drive_pattern',
        description='Movement pattern each robot starts on before any /pattern_command switch',
        choices=PATTERN_CHOICES
    )

    return LaunchDescription([
        robot_count_arg,
        robot_id_arg,
        initial_pattern_arg,
        OpaqueFunction(function=launch_setup)
    ])
