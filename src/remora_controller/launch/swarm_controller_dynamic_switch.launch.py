from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

PATTERN_CHOICES = [
    'drive_pattern', 'random_walk_pattern', 'dispersion_pattern',
    'aggregation_pattern', 'attraction_pattern', 'attraction_pattern2',
    'minimalist_flocking_pattern', 'adaptive_sampling_pattern'
]


def launch_setup(context, *args, **kwargs):
    initial_pattern = LaunchConfiguration('initial_pattern').perform(context)
    robot_count = int(LaunchConfiguration('robot_count').perform(context))

    hardware_protection_param_file = PathJoinSubstitution([
        FindPackageShare('remora_controller'),
        'config',
        'swarm',
        'burger',
        'hardware_protection_layer.yaml'
    ])

    actions = []

    for i in range(robot_count):
        namespace = f'remora{i}'

        # Hardware protection node stays up the whole time, unaffected by pattern switching.
        actions.append(Node(
            package="ros2swarm",
            executable='hardware_protection_layer',
            namespace=namespace,
            parameters=[hardware_protection_param_file],
            remappings=[
                ('range_data', 'obstacle_ranges')
            ],
            output="screen"
        ))

    # Reports each robot's current pattern on /remoraN/current_pattern for the simulator UI.
    actions.append(Node(
        package="remora_controller",
        executable='pattern_status_publisher',
        parameters=[{
            'robot_count': robot_count,
            'initial_pattern': initial_pattern,
        }],
        output="screen"
    ))

    # Owns the actual ros2swarm pattern process per robot; switches on /pattern_command.
    actions.append(Node(
        package="remora_controller",
        executable='pattern_supervisor',
        parameters=[{
            'robot_count': robot_count,
            'initial_pattern': initial_pattern,
        }],
        output="screen"
    ))

    return actions


def generate_launch_description():
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots in the swarm',
    )

    initial_pattern_arg = DeclareLaunchArgument(
        'initial_pattern',
        default_value='drive_pattern',
        description='Movement pattern each robot starts on before any /pattern_command switch',
        choices=PATTERN_CHOICES
    )

    return LaunchDescription([
        robot_count_arg,
        initial_pattern_arg,
        OpaqueFunction(function=launch_setup)
    ])
