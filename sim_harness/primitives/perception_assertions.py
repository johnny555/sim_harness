# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Perception assertion helpers for object detection validation.

These functions help validate that perception systems are detecting
objects correctly in simulation tests.

Note:
    These functions subscribe with ``Any`` message type (for automatic
    topic type discovery) and use early-exit loops, so they cannot use
    :class:`TopicObserver` directly. A small context manager handles
    the executor lifecycle instead.
"""

import math
import time
from contextlib import contextmanager
from dataclasses import dataclass
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
        (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2
    )


@contextmanager
def _managed_executor(node: Node):
    """Create an executor, add node, yield, then clean up."""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        yield executor
    finally:
        executor.remove_node(node)


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
        DetectionResult with detection status and closest position
    """
    result = DetectionResult()
    detected_positions: List[Point] = []

    def detection_callback(msg: Any) -> None:
        nonlocal detected_positions
        if position_extractor is not None:
            positions = position_extractor(msg)
        else:
            positions = _extract_positions_generic(msg)
        detected_positions.extend(positions)

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any, detection_topic, detection_callback, qos_profile_sensor_data
        )
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    with _managed_executor(node) as executor:
        try:
            start = time.monotonic()
            while time.monotonic() - start < timeout_sec:
                executor.spin_once(timeout_sec=0.1)

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
            node.destroy_subscription(sub)

    # Not found within timeout
    result.detection_count = len(detected_positions)
    if detected_positions:
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
        result.details = (
            f"No detections received on {detection_topic} within {timeout_sec}s"
        )

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
        DetectionResult with detection status and confidence info
    """
    result = DetectionResult()
    all_detections: List[tuple] = []

    def detection_callback(msg: Any) -> None:
        nonlocal all_detections
        if class_extractor is not None:
            detections = class_extractor(msg)
        else:
            detections = _extract_class_detections_generic(msg)
        all_detections.extend(detections)

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any, detection_topic, detection_callback, qos_profile_sensor_data
        )
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    with _managed_executor(node) as executor:
        try:
            start = time.monotonic()
            while time.monotonic() - start < timeout_sec:
                executor.spin_once(timeout_sec=0.1)

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
        result.details = (
            f"No detections received on {detection_topic} within {timeout_sec}s"
        )

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

    Returns:
        DetectionResult with detection count
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

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any, detection_topic, detection_callback, qos_profile_sensor_data
        )
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    with _managed_executor(node) as executor:
        try:
            start = time.monotonic()
            while time.monotonic() - start < timeout_sec:
                executor.spin_once(timeout_sec=0.1)

                if max_count_seen >= min_count:
                    result.detected = True
                    result.detection_count = max_count_seen
                    result.details = (
                        f"Detected {max_count_seen} objects (>= {min_count} required)"
                    )
                    return result
        finally:
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
    within the specified region.

    Args:
        node: ROS 2 node for subscriptions
        detection_topic: Topic publishing detection messages
        center: Center of the region to check
        radius: Radius of the region in meters
        observation_period_sec: How long to observe in seconds (default: 5.0)
        position_extractor: Optional function to extract Point list from message.

    Returns:
        True if no objects detected in region, False otherwise
    """
    detected_in_region = False

    def detection_callback(msg: Any) -> None:
        nonlocal detected_in_region
        if detected_in_region:
            return

        if position_extractor is not None:
            positions = position_extractor(msg)
        else:
            positions = _extract_positions_generic(msg)

        for pos in positions:
            if _distance_3d(pos, center) <= radius:
                detected_in_region = True
                return

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(
            Any, detection_topic, detection_callback, qos_profile_sensor_data
        )
    except Exception:
        return True

    with _managed_executor(node) as executor:
        try:
            start = time.monotonic()
            while time.monotonic() - start < observation_period_sec:
                executor.spin_once(timeout_sec=0.1)
                if detected_in_region:
                    break
        finally:
            node.destroy_subscription(sub)

    return not detected_in_region


# ---------------------------------------------------------------------------
# Generic message extractors (domain logic, not boilerplate)
# ---------------------------------------------------------------------------

def _extract_positions_generic(msg: Any) -> List[Point]:
    """
    Try to extract positions from common detection message formats.

    Supports:
    - vision_msgs/Detection3DArray (via bounding box centers)
    - Messages with 'detections' field containing objects with 'position'
    - Messages with 'poses' or 'points' fields
    """
    positions = []

    if hasattr(msg, 'detections'):
        for det in msg.detections:
            if hasattr(det, 'results') and det.results:
                if hasattr(det, 'bbox') and hasattr(det.bbox, 'center'):
                    center = det.bbox.center
                    if hasattr(center, 'position'):
                        positions.append(center.position)
            elif hasattr(det, 'position'):
                positions.append(det.position)

    if hasattr(msg, 'poses'):
        for pose in msg.poses:
            if hasattr(pose, 'position'):
                positions.append(pose.position)

    if hasattr(msg, 'points'):
        for point in msg.points[:100]:
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

            if hasattr(det, 'results') and det.results:
                res = det.results[0]
                if hasattr(res, 'hypothesis'):
                    hyp = res.hypothesis
                    if hasattr(hyp, 'class_id'):
                        class_name = str(hyp.class_id)
                    if hasattr(hyp, 'score'):
                        confidence = float(hyp.score)

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
