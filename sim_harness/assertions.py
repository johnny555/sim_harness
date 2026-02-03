# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Core assertion functions for ROS 2 simulation testing.

Service checks, sensor validation, timing, and vehicle motion assertions.
"""

import math
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple, Type, TypeVar

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from sim_harness.spin import spin_for_duration, spin_until_condition

MsgT = TypeVar('MsgT')

# ── Result types ──────────────────────────────────────────────────────────


@dataclass
class ServiceResult:
    available: bool = False
    call_succeeded: bool = False
    response_time_ms: float = 0.0
    details: str = ""


@dataclass
class SensorDataResult:
    valid: bool = False
    message_count: int = 0
    publish_rate_hz: float = 0.0
    details: str = ""


@dataclass
class TimingResult:
    within_bounds: bool = False
    measured_rate_hz: float = 0.0
    min_latency_ms: float = 0.0
    max_latency_ms: float = 0.0
    avg_latency_ms: float = 0.0
    details: str = ""


@dataclass
class MovementResult:
    success: bool = False
    distance_moved: float = 0.0
    start_position: Tuple[float, float, float] = (0, 0, 0)
    end_position: Tuple[float, float, float] = (0, 0, 0)
    details: str = ""
    ground_truth_distance: Optional[float] = None
    ground_truth_start: Optional[Tuple[float, float, float]] = None
    ground_truth_end: Optional[Tuple[float, float, float]] = None
    odom_error: Optional[float] = None


@dataclass
class VelocityResult:
    success: bool = False
    measured_velocity: float = 0.0
    details: str = ""


# ── Internal helpers ──────────────────────────────────────────────────────


def _collect(node: Node, topic: str, msg_type, timeout_sec: float, qos=10) -> list:
    """Subscribe, spin for *timeout_sec*, return collected messages."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    msgs: list = []
    sub = node.create_subscription(msg_type, topic, msgs.append, qos)
    try:
        spin_for_duration(executor, timeout_sec)
        return msgs
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)


def _pos(msg) -> Tuple[float, float, float]:
    p = msg.pose.pose.position
    return (p.x, p.y, p.z)


def _dist3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _speed(msg) -> float:
    t = msg.twist.twist.linear
    return math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)


# ── Service checks ────────────────────────────────────────────────────────


def assert_service_available(
    node: Node, service_name: str, service_type, timeout_sec: float = 10.0,
) -> ServiceResult:
    """Check that a ROS service is available."""
    client = node.create_client(service_type, service_name)
    try:
        available = client.wait_for_service(timeout_sec=timeout_sec)
        return ServiceResult(
            available=available, call_succeeded=available,
            details="" if available else f"Service {service_name} not available",
        )
    finally:
        node.destroy_client(client)


def assert_action_server_available(
    node: Node, action_name: str, action_type, timeout_sec: float = 10.0,
) -> ServiceResult:
    """Check that a ROS action server is available."""
    from rclpy.action import ActionClient
    client = ActionClient(node, action_type, action_name)
    try:
        available = client.wait_for_server(timeout_sec=timeout_sec)
        return ServiceResult(
            available=available, call_succeeded=available,
            details="" if available else f"Action {action_name} not available",
        )
    finally:
        client.destroy()


def assert_node_running(
    node: Node, target_node_name: str, timeout_sec: float = 10.0,
) -> bool:
    """Check that a node is visible on the ROS graph."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    end = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < end:
            names = [n for n, ns in node.get_node_names_and_namespaces()]
            if target_node_name in names:
                return True
            executor.spin_once(timeout_sec=0.5)
        return False
    finally:
        executor.remove_node(node)


def assert_nodes_running(
    node: Node, node_names: List[str], timeout_sec: float = 30.0,
) -> List[Tuple[str, bool]]:
    """Check multiple nodes. Returns list of (name, found) tuples."""
    return [(n, assert_node_running(node, n, timeout_sec)) for n in node_names]


def assert_parameter_exists(
    node: Node, target_node_name: str, parameter_name: str,
    expected_value=None, timeout_sec: float = 10.0,
) -> bool:
    """Check that a parameter exists on a remote node."""
    from rcl_interfaces.srv import GetParameters
    client = node.create_client(
        GetParameters, f'/{target_node_name}/get_parameters',
    )
    try:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False
        req = GetParameters.Request(names=[parameter_name])
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        try:
            future = client.call_async(req)
            spin_until_condition(executor, future.done, timeout_sec)
            if not future.done():
                return False
            resp = future.result()
            if not resp.values:
                return False
            if expected_value is not None:
                val = resp.values[0]
                if hasattr(val, 'string_value') and val.string_value == str(expected_value):
                    return True
                if hasattr(val, 'double_value') and val.double_value == expected_value:
                    return True
                if hasattr(val, 'integer_value') and val.integer_value == expected_value:
                    return True
                if hasattr(val, 'bool_value') and val.bool_value == expected_value:
                    return True
                return False
            return True
        finally:
            executor.remove_node(node)
    finally:
        node.destroy_client(client)


# ── Sensor checks ─────────────────────────────────────────────────────────


def assert_sensor_publishing(
    node: Node, topic: str, expected_rate_hz: float,
    msg_type=None, tolerance_percent: float = 10.0,
    sample_duration_sec: float = 5.0,
) -> SensorDataResult:
    """Check that a topic publishes at roughly *expected_rate_hz*."""
    if msg_type is None:
        from sensor_msgs.msg import LaserScan
        msg_type = LaserScan
    msgs = _collect(node, topic, msg_type, sample_duration_sec)
    rate = len(msgs) / sample_duration_sec if sample_duration_sec > 0 else 0
    tol = expected_rate_hz * tolerance_percent / 100
    ok = abs(rate - expected_rate_hz) <= tol
    return SensorDataResult(
        valid=ok, message_count=len(msgs), publish_rate_hz=rate,
        details=f"Rate {rate:.1f} Hz (expected {expected_rate_hz:.1f} +/- {tol:.1f})",
    )


def assert_lidar_valid(
    node: Node, topic: str, min_range: float = 0.1, max_range: float = 100.0,
    min_points: int = 100, timeout_sec: float = 5.0,
) -> SensorDataResult:
    """Validate LIDAR scans have enough in-range points."""
    from sensor_msgs.msg import LaserScan
    msgs = _collect(node, topic, LaserScan, timeout_sec)
    if not msgs:
        return SensorDataResult(details="No LaserScan messages received")
    bad = 0
    for scan in msgs:
        pts = sum(1 for r in scan.ranges if math.isfinite(r) and min_range <= r <= max_range)
        if pts < min_points:
            bad += 1
    rate = len(msgs) / timeout_sec if timeout_sec > 0 else 0
    ok = bad == 0
    return SensorDataResult(
        valid=ok, message_count=len(msgs), publish_rate_hz=rate,
        details=f"{len(msgs) - bad}/{len(msgs)} scans valid ({min_points}+ pts in [{min_range}, {max_range}])",
    )


def assert_gps_valid(
    node: Node, topic: str, min_lat: float, max_lat: float,
    min_lon: float, max_lon: float, timeout_sec: float = 5.0,
) -> SensorDataResult:
    """Validate GPS fixes are in-bounds and not NaN."""
    from sensor_msgs.msg import NavSatFix
    msgs = _collect(node, topic, NavSatFix, timeout_sec)
    if not msgs:
        return SensorDataResult(details="No NavSatFix messages received")
    for fix in msgs:
        if not (math.isfinite(fix.latitude) and math.isfinite(fix.longitude)):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="GPS contains NaN",
            )
        if not (min_lat <= fix.latitude <= max_lat and min_lon <= fix.longitude <= max_lon):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"GPS out of bounds: ({fix.latitude:.6f}, {fix.longitude:.6f})",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} fixes valid",
    )


def assert_imu_valid(
    node: Node, topic: str, max_acceleration: float = 50.0,
    max_angular_velocity: float = 10.0, timeout_sec: float = 5.0,
) -> SensorDataResult:
    """Validate IMU readings are finite and within bounds."""
    from sensor_msgs.msg import Imu
    msgs = _collect(node, topic, Imu, timeout_sec)
    if not msgs:
        return SensorDataResult(details="No Imu messages received")
    for imu in msgs:
        a = imu.linear_acceleration
        g = imu.angular_velocity
        vals = [a.x, a.y, a.z, g.x, g.y, g.z]
        if not all(math.isfinite(v) for v in vals):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="IMU contains NaN",
            )
        if max(abs(a.x), abs(a.y), abs(a.z)) > max_acceleration:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Acceleration exceeds {max_acceleration} m/s^2",
            )
        if max(abs(g.x), abs(g.y), abs(g.z)) > max_angular_velocity:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Angular velocity exceeds {max_angular_velocity} rad/s",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} IMU msgs valid",
    )


def assert_camera_valid(
    node: Node, topic: str, expected_width: int = 0, expected_height: int = 0,
    expected_encoding: str = "", timeout_sec: float = 5.0,
) -> SensorDataResult:
    """Validate camera images have data and optional dimension/encoding checks."""
    from sensor_msgs.msg import Image
    msgs = _collect(node, topic, Image, timeout_sec)
    if not msgs:
        return SensorDataResult(details="No Image messages received")
    for img in msgs:
        if len(img.data) == 0:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="Empty image data",
            )
        if expected_width and img.width != expected_width:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Width {img.width} != {expected_width}",
            )
        if expected_height and img.height != expected_height:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Height {img.height} != {expected_height}",
            )
        if expected_encoding and img.encoding != expected_encoding:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Encoding {img.encoding!r} != {expected_encoding!r}",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} images valid",
    )


def assert_joint_states_valid(
    node: Node, topic: str, expected_joints: List[str], timeout_sec: float = 5.0,
) -> SensorDataResult:
    """Validate joint state messages contain expected joints with finite values."""
    from sensor_msgs.msg import JointState
    msgs = _collect(node, topic, JointState, timeout_sec)
    if not msgs:
        return SensorDataResult(details="No JointState messages received")
    for js in msgs:
        missing = [j for j in expected_joints if j not in js.name]
        if missing:
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details=f"Missing joints: {missing}",
            )
        if not all(math.isfinite(v) for v in js.position):
            return SensorDataResult(
                message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
                details="Joint positions contain NaN",
            )
    return SensorDataResult(
        valid=True, message_count=len(msgs), publish_rate_hz=len(msgs) / timeout_sec,
        details=f"{len(msgs)} joint state msgs valid",
    )


# ── Timing checks ─────────────────────────────────────────────────────────


def assert_publish_rate(
    node: Node, topic: str, msg_type: Type[MsgT], expected_rate_hz: float,
    tolerance_percent: float = 10.0, sample_duration_sec: float = 5.0,
) -> TimingResult:
    """Measure topic publish rate and check it matches expected."""
    timestamps: List[float] = []
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def on_msg(_msg):
        timestamps.append(time.monotonic())

    sub = node.create_subscription(msg_type, topic, on_msg, 10)
    try:
        spin_for_duration(executor, sample_duration_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if len(timestamps) < 2:
        return TimingResult(details="Not enough messages to compute rate")

    intervals = [timestamps[i + 1] - timestamps[i] for i in range(len(timestamps) - 1)]
    avg_interval = sum(intervals) / len(intervals)
    rate = 1.0 / avg_interval if avg_interval > 0 else 0
    tol = expected_rate_hz * tolerance_percent / 100
    ok = abs(rate - expected_rate_hz) <= tol
    return TimingResult(
        within_bounds=ok, measured_rate_hz=rate,
        details=f"Rate {rate:.1f} Hz (expected {expected_rate_hz:.1f} +/- {tol:.1f})",
    )


def assert_latency(
    node: Node, topic: str, msg_type: Type[MsgT],
    max_latency_ms: float, sample_duration_sec: float = 5.0,
) -> TimingResult:
    """Measure message latency (header stamp vs receive time)."""
    latencies: List[float] = []
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def on_msg(msg):
        if hasattr(msg, 'header'):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            now = node.get_clock().now().nanoseconds * 1e-9
            latencies.append((now - stamp) * 1000)

    sub = node.create_subscription(msg_type, topic, on_msg, 10)
    try:
        spin_for_duration(executor, sample_duration_sec)
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)

    if not latencies:
        return TimingResult(details="No messages with header received")

    mn, mx, avg = min(latencies), max(latencies), sum(latencies) / len(latencies)
    ok = mx <= max_latency_ms
    return TimingResult(
        within_bounds=ok, min_latency_ms=mn, max_latency_ms=mx, avg_latency_ms=avg,
        details=f"Latency min={mn:.1f} max={mx:.1f} avg={avg:.1f} ms (limit {max_latency_ms})",
    )


def assert_transform_available(
    node: Node, target_frame: str, source_frame: str,
    timeout_sec: float = 5.0, max_age_ms: float = 1000.0,
) -> bool:
    """Check that a TF transform is available and recent."""
    try:
        from tf2_ros import Buffer, TransformListener
    except ImportError:
        return False
    buf = Buffer()
    _ = TransformListener(buf, node)  # Listener starts background thread automatically
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
            try:
                t = buf.lookup_transform(target_frame, source_frame, rclpy.time.Time())
                age_ms = (node.get_clock().now().nanoseconds -
                          rclpy.time.Time.from_msg(t.header.stamp).nanoseconds) / 1e6
                if age_ms <= max_age_ms:
                    return True
            except Exception:
                # Ignore transient transform lookup failures and keep trying until timeout.
                pass
        return False
    finally:
        executor.remove_node(node)


def assert_action_server_responsive(
    node: Node, action_name: str, action_type,
    max_response_time_ms: float = 1000.0,
) -> bool:
    """Check that an action server accepts a goal within time limit."""
    from rclpy.action import ActionClient
    client = ActionClient(node, action_type, action_name)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        if not client.wait_for_server(timeout_sec=max_response_time_ms / 1000):
            return False
        goal = action_type.Goal()
        start = time.monotonic()
        future = client.send_goal_async(goal)
        spin_until_condition(executor, future.done, max_response_time_ms / 1000)
        elapsed = (time.monotonic() - start) * 1000
        return future.done() and elapsed <= max_response_time_ms
    finally:
        client.destroy()
        executor.remove_node(node)


# ── Vehicle motion checks ─────────────────────────────────────────────────


def assert_vehicle_moved(
    node: Node, vehicle_id: str, min_distance: float, velocity: float = 1.0,
    timeout_sec: float = 10.0, odom_topic: Optional[str] = None,
    cmd_vel_topic: Optional[str] = None, use_twist_stamped: bool = True,
) -> MovementResult:
    """Publish a velocity command and verify the robot moves."""
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist, TwistStamped

    otopic = odom_topic or f"/{vehicle_id}/odom"
    ctopic = cmd_vel_topic or f"/{vehicle_id}/cmd_vel"

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    positions: list = []
    sub = node.create_subscription(Odometry, otopic, lambda m: positions.append(_pos(m)), 10)
    pub_type = TwistStamped if use_twist_stamped else Twist
    pub = node.create_publisher(pub_type, ctopic, 10)

    try:
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            if use_twist_stamped:
                cmd = TwistStamped()
                cmd.header.stamp = node.get_clock().now().to_msg()
                cmd.twist.linear.x = velocity
            else:
                cmd = Twist()
                cmd.linear.x = velocity
            pub.publish(cmd)
            executor.spin_once(timeout_sec=0.1)

        # Stop
        stop = TwistStamped() if use_twist_stamped else Twist()
        if use_twist_stamped:
            stop.header.stamp = node.get_clock().now().to_msg()
        pub.publish(stop)
        spin_for_duration(executor, 0.5)

        if len(positions) < 2:
            return MovementResult(details="Not enough odometry messages")
        start, final = positions[0], positions[-1]
        d = _dist3(start, final)
        return MovementResult(
            success=d >= min_distance, distance_moved=d,
            start_position=start, end_position=final,
            details=f"Moved {d:.3f}m (need {min_distance:.3f}m)",
        )
    finally:
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)
        executor.remove_node(node)


def assert_vehicle_stationary(
    node: Node, vehicle_id: str, velocity_threshold: float = 0.01,
    duration_sec: float = 2.0, odom_topic: Optional[str] = None,
) -> bool:
    """Check that the robot is nearly stationary over a duration."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    speeds: list = []
    msgs = _collect(node, otopic, Odometry, duration_sec)
    return all(_speed(m) <= velocity_threshold for m in msgs) if msgs else False


def assert_vehicle_velocity(
    node: Node, vehicle_id: str, target_velocity: float, tolerance: float = 0.1,
    timeout_sec: float = 5.0, odom_topic: Optional[str] = None,
) -> VelocityResult:
    """Check that observed velocity is within tolerance of target."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, timeout_sec)
    if not msgs:
        return VelocityResult(details="No odometry messages")
    speeds = [_speed(m) for m in msgs]
    avg = sum(speeds) / len(speeds)
    ok = abs(avg - target_velocity) <= tolerance
    return VelocityResult(
        success=ok, measured_velocity=avg,
        details=f"Avg velocity {avg:.3f} m/s (target {target_velocity:.3f} +/- {tolerance})",
    )


def assert_vehicle_in_region(
    node: Node, vehicle_id: str,
    min_bounds: Tuple[float, float, float],
    max_bounds: Tuple[float, float, float],
    timeout_sec: float = 5.0, odom_topic: Optional[str] = None,
) -> bool:
    """Check that the robot stays within a bounding box."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, timeout_sec)
    if not msgs:
        return False
    for m in msgs:
        p = _pos(m)
        for i in range(3):
            if not (min_bounds[i] <= p[i] <= max_bounds[i]):
                return False
    return True


def assert_vehicle_orientation(
    node: Node, vehicle_id: str, expected_yaw: float,
    tolerance_rad: float = 0.1, timeout_sec: float = 5.0,
    odom_topic: Optional[str] = None,
) -> bool:
    """Check that the robot's yaw matches expected within tolerance."""
    from nav_msgs.msg import Odometry
    otopic = odom_topic or f"/{vehicle_id}/odom"
    msgs = _collect(node, otopic, Odometry, timeout_sec)
    if not msgs:
        return False
    q = msgs[-1].pose.pose.orientation
    actual = _yaw(q)
    diff = abs(math.atan2(math.sin(actual - expected_yaw), math.cos(actual - expected_yaw)))
    return diff <= tolerance_rad


def assert_vehicle_moved_with_ground_truth(
    node: Node, vehicle_id: str, gazebo_model_name: str,
    min_distance: float, velocity: float = 1.0, timeout_sec: float = 10.0,
    odom_topic: Optional[str] = None, cmd_vel_topic: Optional[str] = None,
    use_twist_stamped: bool = True, world_name: str = "empty",
    odom_tolerance: float = 0.5,
) -> MovementResult:
    """Like assert_vehicle_moved but also compares against Gazebo ground truth."""
    result = assert_vehicle_moved(
        node, vehicle_id, min_distance, velocity, timeout_sec,
        odom_topic, cmd_vel_topic, use_twist_stamped,
    )
    try:
        from sim_harness.simulator.gazebo_ground_truth import GazeboGroundTruth
        gt = GazeboGroundTruth(node, world_name=world_name)
        gt_pos = gt.get_model_position(gazebo_model_name)
        if gt_pos:
            result.ground_truth_end = gt_pos
            if result.ground_truth_start:
                result.ground_truth_distance = _dist3(result.ground_truth_start, gt_pos)
                result.odom_error = abs(result.distance_moved - result.ground_truth_distance)
                if result.odom_error > odom_tolerance:
                    result.details += f"; odom error {result.odom_error:.3f}m > {odom_tolerance}m"
    except ImportError:
        result.details += "; ground truth unavailable (gazebo_ground_truth not installed)"
    return result
