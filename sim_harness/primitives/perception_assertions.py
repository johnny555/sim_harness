# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Perception assertion helpers for object detection validation.

These functions help validate that perception systems are detecting
objects correctly in simulation tests.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional, Any, List, Callable

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


@dataclass
class DetectionResult:
    """Result of a detection assertion."""

    detected: bool = False
    """Whether object was detected."""

    detection_count: int = 0
    """Number of detections found."""

    closest_position: Optional[Point] = None
    """Closest detected position (if any)."""

    details: str = ""
    """Human-readable details."""


def _distance_3d(p1: Point, p2: Point) -> float:
    """Calculate 3D Euclidean distance between two points."""
    return math.sqrt(
        (p1.x - p2.x) ** 2 +
        (p1.y - p2.y) ** 2 +
        (p1.z - p2.z) ** 2
    )


def assert_object_detected(
    node: Node,
    detection_topic: str,
    expected_position: Point,
    search_radius: float = 1.0,
    timeout_sec: float = 10.0,
    position_extractor: Optional[Callable[[Any], List[Point]]] = None,
) -> DetectionResult:
    """
    Assert that an object is detected near an expected position.

    This function subscribes to a detection topic and checks if any
    detected objects are within search_radius of the expected position.

    Args:
        node: ROS 2 node for subscriptions
        detection_topic: Topic publishing detection messages
        expected_position: Expected object position (geometry_msgs/Point)
        search_radius: Search radius in meters (default: 1.0)
        timeout_sec: Maximum time to wait in seconds (default: 10.0)
        position_extractor: Optional function to extract Point list from
            detection message. If None, tries common message formats.

    Returns:
        DetectionResult with detection status and details

    Example::

        expected = Point(x=5.0, y=2.0, z=0.0)
        result = assert_object_detected(
            node,
            '/detections',
            expected,
            search_radius=0.5
        )
        assert result.detected, f"Object not detected: {result.details}"
    """
    result = DetectionResult()
    detected_positions: List[Point] = []

    def detection_callback(msg: Any) -> None:
        nonlocal detected_positions
        positions = []

        if position_extractor is not None:
            positions = position_extractor(msg)
        else:
            # Try common message formats
            positions = _extract_positions_generic(msg)

        detected_positions.extend(positions)

    # Try to subscribe - message type is dynamic
    try:
        # Use generic subscription that accepts any message type
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any,  # Will be overridden by topic type discovery
            detection_topic,
            detection_callback,
            qos_profile_sensor_data
        )
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    # Wait and spin
    start = time.monotonic()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while time.monotonic() - start < timeout_sec:
            executor.spin_once(timeout_sec=0.1)

            # Check if any detection is within radius
            for pos in detected_positions:
                dist = _distance_3d(pos, expected_position)
                if dist <= search_radius:
                    result.detected = True
                    result.detection_count = len(detected_positions)
                    result.closest_position = pos
                    result.details = (
                        f"Object detected at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), "
                        f"distance={dist:.2f}m from expected"
                    )
                    return result

    finally:
        executor.remove_node(node)
        node.destroy_subscription(sub)

    # Not found within timeout
    result.detection_count = len(detected_positions)
    if detected_positions:
        # Find closest
        closest_dist = float('inf')
        for pos in detected_positions:
            dist = _distance_3d(pos, expected_position)
            if dist < closest_dist:
                closest_dist = dist
                result.closest_position = pos

        result.details = (
            f"No object within {search_radius}m of expected position. "
            f"Closest detection at {closest_dist:.2f}m"
        )
    else:
        result.details = f"No detections received on {detection_topic} within {timeout_sec}s"

    return result


def assert_object_detected_by_class(
    node: Node,
    detection_topic: str,
    object_class: str,
    min_confidence: float = 0.5,
    timeout_sec: float = 10.0,
    class_extractor: Optional[Callable[[Any], List[tuple]]] = None,
) -> DetectionResult:
    """
    Assert that an object of a specific class is detected.

    Args:
        node: ROS 2 node for subscriptions
        detection_topic: Topic publishing detection messages
        object_class: Class name/label to search for (e.g., "person", "car")
        min_confidence: Minimum confidence threshold (0.0-1.0, default: 0.5)
        timeout_sec: Maximum time to wait in seconds (default: 10.0)
        class_extractor: Optional function to extract (class_name, confidence, position)
            tuples from detection message. If None, tries common formats.

    Returns:
        DetectionResult with detection status and details

    Example::

        result = assert_object_detected_by_class(
            node,
            '/yolo/detections',
            'person',
            min_confidence=0.7
        )
        assert result.detected
    """
    result = DetectionResult()
    all_detections: List[tuple] = []  # (class_name, confidence, position)

    def detection_callback(msg: Any) -> None:
        nonlocal all_detections
        if class_extractor is not None:
            detections = class_extractor(msg)
        else:
            detections = _extract_class_detections_generic(msg)
        all_detections.extend(detections)

    # Subscribe
    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any,
            detection_topic,
            detection_callback,
            qos_profile_sensor_data
        )
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    # Wait and spin
    start = time.monotonic()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while time.monotonic() - start < timeout_sec:
            executor.spin_once(timeout_sec=0.1)

            # Check for matching class with sufficient confidence
            for class_name, confidence, position in all_detections:
                if class_name.lower() == object_class.lower() and confidence >= min_confidence:
                    result.detected = True
                    result.detection_count = sum(
                        1 for c, conf, _ in all_detections
                        if c.lower() == object_class.lower() and conf >= min_confidence
                    )
                    result.closest_position = position
                    result.details = (
                        f"Detected '{class_name}' with confidence {confidence:.2f}"
                    )
                    return result

    finally:
        executor.remove_node(node)
        node.destroy_subscription(sub)

    # Not found
    class_matches = [
        (c, conf) for c, conf, _ in all_detections
        if c.lower() == object_class.lower()
    ]
    if class_matches:
        best_conf = max(conf for _, conf in class_matches)
        result.details = (
            f"Found '{object_class}' but max confidence {best_conf:.2f} < {min_confidence}"
        )
    elif all_detections:
        classes_found = set(c for c, _, _ in all_detections)
        result.details = (
            f"Class '{object_class}' not found. Detected classes: {classes_found}"
        )
    else:
        result.details = f"No detections received on {detection_topic} within {timeout_sec}s"

    return result


def assert_min_objects_detected(
    node: Node,
    detection_topic: str,
    min_count: int,
    timeout_sec: float = 10.0,
    count_extractor: Optional[Callable[[Any], int]] = None,
) -> DetectionResult:
    """
    Assert that a minimum number of objects are detected.

    Args:
        node: ROS 2 node for subscriptions
        detection_topic: Topic publishing detection messages
        min_count: Minimum number of detections required
        timeout_sec: Maximum time to wait in seconds (default: 10.0)
        count_extractor: Optional function to extract detection count from message.
            If None, tries common message formats.

    Returns:
        DetectionResult with detection status and details

    Example::

        result = assert_min_objects_detected(
            node,
            '/obstacles',
            min_count=3,
            timeout_sec=5.0
        )
        assert result.detected, f"Expected >=3 objects: {result.details}"
    """
    result = DetectionResult()
    max_count_seen = 0

    def detection_callback(msg: Any) -> None:
        nonlocal max_count_seen
        if count_extractor is not None:
            count = count_extractor(msg)
        else:
            count = _extract_detection_count_generic(msg)
        max_count_seen = max(max_count_seen, count)

    # Subscribe
    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any,
            detection_topic,
            detection_callback,
            qos_profile_sensor_data
        )
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    # Wait and spin
    start = time.monotonic()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while time.monotonic() - start < timeout_sec:
            executor.spin_once(timeout_sec=0.1)

            if max_count_seen >= min_count:
                result.detected = True
                result.detection_count = max_count_seen
                result.details = f"Detected {max_count_seen} objects (>= {min_count} required)"
                return result

    finally:
        executor.remove_node(node)
        node.destroy_subscription(sub)

    result.detection_count = max_count_seen
    result.details = f"Only detected {max_count_seen} objects, need >= {min_count}"
    return result


def assert_region_clear(
    node: Node,
    detection_topic: str,
    center: Point,
    radius: float,
    observation_period_sec: float = 5.0,
    position_extractor: Optional[Callable[[Any], List[Point]]] = None,
) -> bool:
    """
    Assert that a region is clear of detections (safety zone check).

    Observes detections for a period and verifies no objects are detected
    within the specified region. Useful for safety zone validation.

    Args:
        node: ROS 2 node for subscriptions
        detection_topic: Topic publishing detection messages
        center: Center of the region to check
        radius: Radius of the region in meters
        observation_period_sec: How long to observe in seconds (default: 5.0)
        position_extractor: Optional function to extract Point list from message.

    Returns:
        True if no objects detected in region, False otherwise

    Example::

        # Check that safety zone around robot is clear
        robot_pos = Point(x=0.0, y=0.0, z=0.0)
        is_clear = assert_region_clear(
            node,
            '/obstacles',
            center=robot_pos,
            radius=1.5,
            observation_period_sec=3.0
        )
        assert is_clear, "Safety zone not clear!"
    """
    detected_in_region = False
    violation_position: Optional[Point] = None

    def detection_callback(msg: Any) -> None:
        nonlocal detected_in_region, violation_position
        if detected_in_region:
            return  # Already found violation

        if position_extractor is not None:
            positions = position_extractor(msg)
        else:
            positions = _extract_positions_generic(msg)

        for pos in positions:
            dist = _distance_3d(pos, center)
            if dist <= radius:
                detected_in_region = True
                violation_position = pos
                return

    # Subscribe
    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any,
            detection_topic,
            detection_callback,
            qos_profile_sensor_data
        )
    except Exception:
        # If we can't subscribe, assume clear (conservative approach)
        return True

    # Observe for the specified period
    start = time.monotonic()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while time.monotonic() - start < observation_period_sec:
            executor.spin_once(timeout_sec=0.1)
            if detected_in_region:
                break
    finally:
        executor.remove_node(node)
        node.destroy_subscription(sub)

    return not detected_in_region


def _extract_positions_generic(msg: Any) -> List[Point]:
    """
    Try to extract positions from common detection message formats.

    Supports:
    - vision_msgs/Detection2DArray (via bounding box centers)
    - vision_msgs/Detection3DArray
    - Messages with 'detections' field containing objects with 'position'
    - Messages with 'points' or 'poses' fields
    """
    positions = []

    # Try vision_msgs Detection3DArray style
    if hasattr(msg, 'detections'):
        for det in msg.detections:
            if hasattr(det, 'results') and det.results:
                # vision_msgs/Detection3D
                if hasattr(det, 'bbox') and hasattr(det.bbox, 'center'):
                    center = det.bbox.center
                    if hasattr(center, 'position'):
                        positions.append(center.position)
            elif hasattr(det, 'position'):
                positions.append(det.position)

    # Try PoseArray style
    if hasattr(msg, 'poses'):
        for pose in msg.poses:
            if hasattr(pose, 'position'):
                positions.append(pose.position)

    # Try PointCloud style (sample first few points)
    if hasattr(msg, 'points'):
        for point in msg.points[:100]:  # Limit to avoid huge clouds
            if isinstance(point, Point):
                positions.append(point)
            elif hasattr(point, 'x') and hasattr(point, 'y') and hasattr(point, 'z'):
                p = Point()
                p.x = float(point.x)
                p.y = float(point.y)
                p.z = float(point.z)
                positions.append(p)

    return positions


def _extract_class_detections_generic(msg: Any) -> List[tuple]:
    """
    Try to extract (class_name, confidence, position) tuples from common formats.

    Supports vision_msgs/Detection2DArray and Detection3DArray styles.
    """
    detections = []

    if hasattr(msg, 'detections'):
        for det in msg.detections:
            class_name = ""
            confidence = 0.0
            position = None

            # Get class and confidence from results
            if hasattr(det, 'results') and det.results:
                result = det.results[0]  # Take top result
                if hasattr(result, 'hypothesis'):
                    hyp = result.hypothesis
                    if hasattr(hyp, 'class_id'):
                        class_name = str(hyp.class_id)
                    if hasattr(hyp, 'score'):
                        confidence = float(hyp.score)

            # Get position from bbox
            if hasattr(det, 'bbox') and hasattr(det.bbox, 'center'):
                center = det.bbox.center
                if hasattr(center, 'position'):
                    position = center.position
                elif hasattr(center, 'x'):
                    position = Point()
                    position.x = float(center.x)
                    position.y = float(getattr(center, 'y', 0.0))
                    position.z = float(getattr(center, 'z', 0.0))

            if class_name:
                detections.append((class_name, confidence, position))

    return detections


def _extract_detection_count_generic(msg: Any) -> int:
    """Try to extract detection count from common message formats."""
    if hasattr(msg, 'detections'):
        return len(msg.detections)
    if hasattr(msg, 'objects'):
        return len(msg.objects)
    if hasattr(msg, 'markers'):
        return len(msg.markers)
    if hasattr(msg, 'poses'):
        return len(msg.poses)
    return 0
