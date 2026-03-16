#!/usr/bin/env python3

"""
TF Relay Node.

Relays TF transforms from a namespaced topic to the global /tf topic. This is
necessary when using gz_ros2_control with a namespace, which causes the steering
controller to publish odometry TF to a namespaced topic (e.g.
/{ns}/steering_controller/tf_odometry) instead of global /tf.

Parameters:
    source_topic (str): Topic to subscribe to (default: /yt220_01/steering_controller/tf_odometry)
    target_topic (str): Topic to publish to (default: /tf)
"""

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')

        # Declare parameters
        self.declare_parameter('source_topic', '/yt220_01/steering_controller/tf_odometry')
        self.declare_parameter('target_topic', '/tf')

        # Get parameters
        source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        target_topic = self.get_parameter('target_topic').get_parameter_value().string_value

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
