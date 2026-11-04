"""Launch the LSTM fault detector using a Python interpreter that has TensorFlow.

The ML dependencies (see requirements.txt) are expected to live in a virtualenv.
Point to it with either:
    export ML_VENV=/path/to/ml_venv
    ros2 launch fault_detector lstm.launch.py
or:
    ros2 launch fault_detector lstm.launch.py ml_venv:=/path/to/ml_venv

If neither is given, the system python3 is used.

The ROS workspace must be sourced first (source install/setup.bash) so that
rclpy and this package are on PYTHONPATH; that PYTHONPATH is passed through
to the venv interpreter.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _launch_detector(context):
    venv = LaunchConfiguration('ml_venv').perform(context)
    fault_type = LaunchConfiguration('fault_type').perform(context)

    python = os.path.join(venv, 'bin', 'python3') if venv else 'python3'

    return [
        ExecuteProcess(
            cmd=[
                python, '-m', 'fault_detector.lstm_fault_detector',
                '--ros-args', '-p', f'fault_type:={fault_type}',
            ],
            additional_env={'TF_CPP_MIN_LOG_LEVEL': '3'},
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ml_venv',
            default_value=os.environ.get('ML_VENV', ''),
            description='Path to the virtualenv containing TensorFlow etc. '
                        '(defaults to $ML_VENV; empty uses system python3)',
        ),
        DeclareLaunchArgument(
            'fault_type',
            default_value='deg',
            description='Fault model to load: deg or if',
        ),
        OpaqueFunction(function=_launch_detector),
    ])
