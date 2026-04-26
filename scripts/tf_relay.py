#!/usr/bin/env python3

"""
TF Relay Node.

Relays TF messages from one topic to another. Useful when a controller (e.g.
gz_ros2_control with a namespace) publishes odometry TF to a namespaced topic
instead of the global /tf.

Parameters:
    source_topic (str): Topic to subscribe to (required — no default)
    target_topic (str): Topic to publish to (default: /tf)
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')

        # Declare parameters. source_topic has no meaningful default — the
        # caller must pass one via ros2 launch / --ros-args -p ...
        self.declare_parameter('source_topic', '')
        self.declare_parameter('target_topic', '/tf')

        # Get parameters
        source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        if not source_topic:
            raise ValueError(
                "tf_relay: 'source_topic' parameter is required "
                "(e.g. -p source_topic:=/<ns>/steering_controller/tf_odometry)"
            )

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            TFMessage,
            source_topic,
            self.tf_callback,
            10
        )

        self.publisher = self.create_publisher(
            TFMessage,
            target_topic,
            10
        )

        self.get_logger().info(f'TF Relay started: {source_topic} -> {target_topic}')

    def tf_callback(self, msg):
        """Relay TF message from source to target topic."""
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
