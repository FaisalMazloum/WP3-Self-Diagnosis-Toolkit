#!/usr/bin/env python3
import ctypes
import signal
import subprocess

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from ros2run.api import get_executable_path
from std_msgs.msg import String

TERMINATE_TIMEOUT_SEC = 5.0
PR_SET_PDEATHSIG = 1


def _die_with_parent():
    # Runs in the child right after fork (Linux only): if this supervisor process
    # ever dies for any reason -- kill -9, crash, ros2 launch force-killing it after
    # its shutdown grace period -- the kernel sends SIGTERM to this child automatically,
    # so patterns never end up orphaned even when our own cleanup code never gets to run.
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGTERM)


class PatternSupervisor(Node):
    """Spawns/kills unmodified ros2swarm pattern executables per robot on command."""

    def __init__(self):
        super().__init__('pattern_supervisor')

        self.declare_parameter('robot_count', 1)
        self.declare_parameter('initial_pattern', 'drive_pattern')

        robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        initial_pattern = self.get_parameter('initial_pattern').get_parameter_value().string_value

        self._status_publisher = self.create_publisher(String, '/pattern_switch_command', 10)
        self._processes = {}
        self._current_pattern = {}

        for i in range(robot_count):
            namespace = f'remora{i}'
            self._start_pattern(namespace, initial_pattern)

        self.create_subscription(String, '/pattern_command', self._on_pattern_command, 10)

        self.get_logger().info(
            f'Supervising {robot_count} robot(s), starting on "{initial_pattern}". '
            f'Publish "namespace:pattern" on /pattern_command to switch.'
        )

    def _on_pattern_command(self, msg):
        try:
            namespace, pattern = msg.data.split(':', 1)
        except ValueError:
            self.get_logger().warn(f'Malformed command: "{msg.data}" (expected "namespace:pattern")')
            return

        if namespace not in self._processes:
            self.get_logger().warn(f'Unknown robot namespace "{namespace}"')
            return

        if self._current_pattern.get(namespace) == pattern:
            self.get_logger().debug(f'{namespace} already on "{pattern}", ignoring duplicate command')
            return

        self._start_pattern(namespace, pattern)

    def _start_pattern(self, namespace, pattern):
        # Spawn the resolved executable directly (bypassing "ros2 run", which would spawn
        # it as a *child* of a ros2 CLI wrapper process -- terminate() would then only
        # kill the wrapper and orphan the real node, leaving it running forever).
        executable_path = get_executable_path(package_name='ros2swarm', executable_name=pattern)
        if executable_path is None:
            self.get_logger().warn(f'Unknown pattern "{pattern}" in ros2swarm, ignoring command')
            return

        self._stop_pattern(namespace)

        param_file = (
            f'{get_package_share_directory("remora_controller")}/config/swarm/burger/'
            f'movement_pattern/basic/{pattern}.yaml'
        )
        cmd = [
            executable_path,
            '--ros-args',
            '-r', f'__ns:=/{namespace}',
            '--params-file', param_file,
        ]
        self._processes[namespace] = subprocess.Popen(cmd, preexec_fn=_die_with_parent)
        self._current_pattern[namespace] = pattern

        status_msg = String()
        status_msg.data = f'{namespace}:{pattern}'
        self._status_publisher.publish(status_msg)

        self.get_logger().info(f'{namespace} -> {pattern}')

    def _stop_pattern(self, namespace):
        process = self._processes.get(namespace)
        if process is None or process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=TERMINATE_TIMEOUT_SEC)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    def destroy_node(self):
        for namespace in list(self._processes):
            self._stop_pattern(namespace)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PatternSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
