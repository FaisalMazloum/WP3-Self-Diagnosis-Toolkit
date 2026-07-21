#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

PUBLISH_RATE_HZ = 1.0


class PatternStatusPublisher(Node):

    def __init__(self):
        super().__init__('pattern_status_publisher')

        self.declare_parameter('robot_count', 1)
        self.declare_parameter('initial_pattern', 'drive_pattern')

        robot_count = self.get_parameter('robot_count').get_parameter_value().integer_value
        initial_pattern = self.get_parameter('initial_pattern').get_parameter_value().string_value

        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._status_publishers = {}
        self._current_pattern = {}
        for i in range(robot_count):
            namespace = f'remora{i}'
            self._status_publishers[namespace] = self.create_publisher(
                String, f'/{namespace}/current_pattern', status_qos
            )
            self._current_pattern[namespace] = initial_pattern

        self.create_subscription(
            String,
            '/pattern_switch_command',
            self._on_switch_command,
            10,
        )

        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_all)

        self.get_logger().info(
            f'Publishing current_pattern for {robot_count} robot(s) at {PUBLISH_RATE_HZ} Hz, '
            f'starting on "{initial_pattern}"'
        )

    def _on_switch_command(self, msg):
        try:
            namespace, pattern = msg.data.split(':', 1)
        except ValueError:
            self.get_logger().warn(f'Malformed switch command: "{msg.data}" (expected "namespace:pattern")')
            return

        if namespace not in self._current_pattern:
            self.get_logger().warn(f'Unknown robot namespace "{namespace}" in switch command')
            return

        self._current_pattern[namespace] = pattern
        self.get_logger().info(f'{namespace} -> {pattern}')

    def _publish_all(self):
        for namespace, pattern in self._current_pattern.items():
            msg = String()
            msg.data = pattern
            self._status_publishers[namespace].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatternStatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
