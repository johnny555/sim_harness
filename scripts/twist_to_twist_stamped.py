#!/usr/bin/env python3

"""
Bridge node to convert Twist to TwistStamped for ros2_control steering controllers.

Nav2 controller_server publishes geometry_msgs/Twist on cmd_vel, but ros2_control's
ackermann_steering_controller (ROS2 Jazzy) only subscribes to
geometry_msgs/TwistStamped on ~/reference. There is no config option to change this —
the bridge is mandatory.

Parameters:
    frame_id (str): Frame ID for the TwistStamped header (default: base_link)

Topics:
    Subscribes: cmd_vel (geometry_msgs/Twist)
    Publishes: cmd_vel_stamped (geometry_msgs/TwistStamped)
"""

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node


class TwistToTwistStampedBridge(Node):
    """Converts Twist messages to TwistStamped messages."""

    def __init__(self):
        super().__init__('twist_to_twist_stamped_bridge')

        # Parameters
        self.declare_parameter('frame_id', 'base_link')
        self.frame_id = self.get_parameter('frame_id').value

        # Subscriber to cmd_vel (Twist)
        self.twist_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )

        # Publisher to steering_controller/reference (TwistStamped)
        self.twist_stamped_pub = self.create_publisher(
            TwistStamped,
            'cmd_vel_stamped',
            10
        )

        self.get_logger().info(
            'Twist to TwistStamped bridge started '
            f'(frame: {self.frame_id})'
        )

    def twist_callback(self, msg):
        """Convert Twist to TwistStamped and republish."""
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = self.frame_id
        twist_stamped.twist = msg

        self.twist_stamped_pub.publish(twist_stamped)


def main(args=None):
    rclpy.init(args=args)
    bridge = TwistToTwistStampedBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
