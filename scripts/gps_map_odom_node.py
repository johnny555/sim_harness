#!/usr/bin/env python3
"""
GPS Map→Odom TF Publisher.

Computes map→odom from GPS position + wheel odometry TF.
Uses wall-clock timer (no use_sim_time) but stamps TF with sim time
from /clock subscription. This avoids the sim_time timer deadlock.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
import tf2_ros


class GpsMapOdomNode(Node):
    def __init__(self):
        super().__init__('gps_map_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('settle_threshold', 0.05)
        self.declare_parameter('settle_count', 20)
        from rcl_interfaces.msg import ParameterDescriptor
        float_desc = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('spawn_x', 0.0, float_desc)
        self.declare_parameter('spawn_y', 0.0, float_desc)
        self.declare_parameter('spawn_yaw', 0.0, float_desc)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        self.settle_threshold = self.get_parameter('settle_threshold').value
        self.settle_count_needed = self.get_parameter('settle_count').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.settled = False
        self.stable_count = 0
        settled_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.settled_pub = self.create_publisher(Bool, 'gps_settled', settled_qos)

        self.gps_x = None
        self.gps_y = None
        self.gps_yaw = None
        self.sim_time = None
        self.log_count = 0

        sx = self.get_parameter('spawn_x').value
        sy = self.get_parameter('spawn_y').value
        syaw = self.get_parameter('spawn_yaw').value
        self.smooth_tx = float(sx)
        self.smooth_ty = float(sy)
        self.smooth_yaw = float(syaw)

        # Subscribe to /clock for sim time stamps (node uses wall clock internally)
        self.create_subscription(Clock, '/clock', self._clock_cb, 10)
        self.create_subscription(Odometry, 'gps/odom', self._gps_cb, 10)

        rate = self.get_parameter('publish_rate').value
        self._timer = self.create_timer(1.0 / rate, self._publish_tf)
        self.get_logger().info(f'Timer created: period={1.0/rate:.3f}s')

        self.get_logger().info(
            f'GPS map->odom: {self.map_frame}->{self.odom_frame}')

    def _clock_cb(self, msg):
        self.sim_time = msg.clock

    def _gps_cb(self, msg):
        self.gps_x = msg.pose.pose.position.x
        self.gps_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.gps_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _publish_tf(self):
        if self.sim_time is None:
            return

        if self.gps_x is None:
            return  # let static TF handle map→odom until GPS arrives

        try:
            odom_tf = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame,
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.0))
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        odom_x = odom_tf.transform.translation.x
        odom_y = odom_tf.transform.translation.y
        if math.isnan(odom_x) or math.isnan(odom_y):
            return
        q = odom_tf.transform.rotation
        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.log_count += 1

        # Full map→odom: position AND yaw correction.
        # Yaw is smoothed (EMA alpha=0.02) to prevent dual-antenna heading jitter
        # from destabilizing the position correction.
        raw_yaw = self.gps_yaw - odom_yaw if self.gps_yaw is not None else self.smooth_yaw
        dyaw = math.atan2(math.sin(raw_yaw - self.smooth_yaw),
                          math.cos(raw_yaw - self.smooth_yaw))
        self.smooth_yaw += 0.02 * dyaw

        cos_yd = math.cos(self.smooth_yaw)
        sin_yd = math.sin(self.smooth_yaw)
        raw_tx = self.gps_x - (cos_yd * odom_x - sin_yd * odom_y)
        raw_ty = self.gps_y - (sin_yd * odom_x + cos_yd * odom_y)
        jump = math.hypot(raw_tx - self.smooth_tx, raw_ty - self.smooth_ty)

        self.smooth_tx = raw_tx
        self.smooth_ty = raw_ty

        if self.log_count % 50 == 0:
            self.get_logger().info(
                f'odom=({odom_x:.2f},{odom_y:.2f}) '
                f'gps=({self.gps_x:.2f},{self.gps_y:.2f}) '
                f'correction=({self.smooth_tx:.2f},{self.smooth_ty:.2f}) '
                f'jump={jump:.3f}m')

        if not self.settled:
            if jump < self.settle_threshold:
                self.stable_count += 1
            else:
                self.stable_count = 0
            if self.stable_count >= self.settle_count_needed:
                self.settled = True
                self.get_logger().info(
                    f'GPS settled: correction=({self.smooth_tx:.3f}, {self.smooth_ty:.3f})')
                self.settled_pub.publish(Bool(data=True))

        self._broadcast(self.smooth_tx, self.smooth_ty, self.smooth_yaw)

    def _broadcast(self, tx, ty, yaw):
        t = TransformStamped()
        t.header.stamp = self.sim_time
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = GpsMapOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
