import random

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, EmitEvent, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.events import matches_action
from launch.events.process import ShutdownProcess

PATTERN_CHOICES = [
    'drive_pattern', 'random_walk_pattern', 'dispersion_pattern',
    'aggregation_pattern', 'attraction_pattern', 'attraction_pattern2',
    'minimalist_flocking_pattern', 'adaptive_sampling_pattern'
]


def movement_param_file(pattern_name):
    return PathJoinSubstitution([
        FindPackageShare('remora_controller'),
        'config',
        'swarm',
        'burger',
        'movement_pattern',
        'basic',
        f'{pattern_name}.yaml'
    ])


def launch_setup(context, *args, **kwargs):
    pattern_a = LaunchConfiguration('pattern_a').perform(context)
    pattern_b = LaunchConfiguration('pattern_b').perform(context)
    robot_count = int(LaunchConfiguration('robot_count').perform(context))
    switch_duration = float(LaunchConfiguration('switch_duration').perform(context))
    stagger_duration = float(LaunchConfiguration('stagger_duration').perform(context))

    hardware_protection_param_file = PathJoinSubstitution([
        FindPackageShare('remora_controller'),
        'config',
        'swarm',
        'burger',
        'hardware_protection_layer.yaml'
    ])

    actions = []

    # Persistent node that continuously republishes each robot's current pattern
    # (/remoraN/current_pattern), so the simulator UI always has an up-to-date value.
    actions.append(Node(
        package="remora_controller",
        executable='pattern_status_publisher',
        parameters=[{
            'robot_count': robot_count,
            'initial_pattern': pattern_a,
        }],
        output="screen"
    ))

    # Random order in which robots take their turn switching from pattern_a to pattern_b.
    switch_order = list(range(robot_count))
    random.shuffle(switch_order)

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

        pattern_a_node = Node(
            package="ros2swarm",
            executable=pattern_a,
            namespace=namespace,
            parameters=[movement_param_file(pattern_a)],
            output="screen"
        )
        actions.append(pattern_a_node)

        pattern_b_node = Node(
            package="ros2swarm",
            executable=pattern_b,
            namespace=namespace,
            parameters=[movement_param_file(pattern_b)],
            output="screen"
        )

        # This robot's turn in the random order: turn 0 switches at switch_duration,
        # turn 1 at switch_duration + stagger_duration, and so on.
        turn = switch_order.index(i)
        switch_time = switch_duration + turn * stagger_duration

        actions.append(TimerAction(
            period=switch_time,
            actions=[
                EmitEvent(event=ShutdownProcess(process_matcher=matches_action(pattern_a_node))),
                pattern_b_node,
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/pattern_switch_command', 'std_msgs/msg/String',
                        f'data: "{namespace}:{pattern_b}"'
                    ],
                    output="screen"
                ),
            ]
        ))

    return actions


def generate_launch_description():
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots in the swarm',
    )

    pattern_a_arg = DeclareLaunchArgument(
        'pattern_a',
        default_value='drive_pattern',
        description='Initial movement pattern, run by every robot until its turn to switch',
        choices=PATTERN_CHOICES
    )

    pattern_b_arg = DeclareLaunchArgument(
        'pattern_b',
        default_value='random_walk_pattern',
        description='Movement pattern each robot switches to, one at a time, running indefinitely after its turn',
        choices=PATTERN_CHOICES
    )

    switch_duration_arg = DeclareLaunchArgument(
        'switch_duration',
        default_value='30.0',
        description='Seconds before the first (randomly chosen) robot switches from pattern_a to pattern_b',
    )

    stagger_duration_arg = DeclareLaunchArgument(
        'stagger_duration',
        default_value='10.0',
        description='Seconds between each subsequent robot switching (in a random, uniformly drawn order) after switch_duration',
    )

    return LaunchDescription([
        robot_count_arg,
        pattern_a_arg,
        pattern_b_arg,
        switch_duration_arg,
        stagger_duration_arg,
        OpaqueFunction(function=launch_setup)
    ])
