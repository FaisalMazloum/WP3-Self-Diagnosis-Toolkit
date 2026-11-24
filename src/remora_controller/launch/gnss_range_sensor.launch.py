"""
Starts one gnss_range_sensor per robot (namespaces remora0 .. remora{N-1}).

Simulation (side by side with Unity's raycast LIDAR, defaults are set for Unity):
    ros2 launch remora_controller gnss_range_sensor.launch.py robot_count:=5
    -> /remoraX/gnss_range_data   vs   /remoraX/range_data (Unity)

Real robot (replace the missing LIDAR), one robot per machine, REP-103 (ENU) IMU:
    ros2 launch remora_controller gnss_range_sensor.launch.py \
        robot_id:=remora3 output_topic:=range_data imu_yaw_offset_deg:=0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    robot_id = arg('robot_id')
    namespaces = [robot_id] if robot_id else [f'remora{i}' for i in range(int(arg('robot_count')))]

    params = {
        'output_topic': arg('output_topic'),
        'robot_radius': float(arg('robot_radius')),
        'use_yaw': arg('use_yaw').lower() in ('true', '1', 'yes'),
        'angle_direction': arg('angle_direction'),
        'imu_yaw_offset_deg': float(arg('imu_yaw_offset_deg')),
    }

    return [
        Node(
            package='remora_controller',
            executable='gnss_range_sensor',
            namespace=ns,
            parameters=[params],
            output='screen',
        )
        for ns in namespaces
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='',
                              description='Single robot namespace (real robot). Overrides robot_count.'),
        DeclareLaunchArgument('robot_count', default_value='1',
                              description='Number of robots remora0..N-1 (simulation)'),
        DeclareLaunchArgument('output_topic', default_value='gnss_range_data',
                              description='Scan topic inside the namespace; use range_data on real robots'),
        DeclareLaunchArgument('robot_radius', default_value='0.225',
                              description='Radius (m) of the disc each neighbour occupies in the scan'),
        DeclareLaunchArgument('use_yaw', default_value='true',
                              description='true: angles relative to heading (needs imu). false: world ENU angles'),
        DeclareLaunchArgument('angle_direction', default_value='cw',
                              description="'cw' = same as Unity RemoraLIDAR (90 deg = right), 'ccw' = REP-103"),
        DeclareLaunchArgument('imu_yaw_offset_deg', default_value='90',
                              description='yaw_enu = yaw_imu + offset. 90 for the Unity IMU (yaw 0 = North), '
                                          '0 for a REP-103 ENU IMU'),
        OpaqueFunction(function=launch_setup),
    ])
