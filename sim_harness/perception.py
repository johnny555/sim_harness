# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Perception extensions -- object detection and region checks.

Primary API uses ``check_*`` names; ``assert_*`` aliases are provided for
backward compatibility.

Example::

    from sim_harness import SimTestFixture
    from sim_harness.perception import check_object_detected, check_region_clear

    class TestPerception(SimTestFixture):
        def test_detects_obstacle(self):
            result = check_object_detected(
                self.node, '/detections', expected_position)
            assert result.ok, result.details
"""

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Optional

from rclpy.node import Node
from geometry_msgs.msg import Point

from sim_harness.spin import ExecutorContext


@dataclass
class DetectionResult:
    detected: bool = False
    detection_count: int = 0
    closest_position: Optional[Point] = None
    details: str = ""

    @property
    def ok(self) -> bool:
        return self.detected


def _dist3(p1: Point, p2: Point) -> float:
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)


# -- Check functions ──────────────────────────────────────────────────────


def check_object_detected(
    node: Node, detection_topic: str, expected_position: Point,
    search_radius: float = 1.0, timeout_sec: float = 10.0,
    position_extractor: Optional[Callable[[Any], List[Point]]] = None,
    executor=None,
) -> DetectionResult:
    """Check that an object is detected near an expected position."""
    result = DetectionResult()
    positions: List[Point] = []

    def cb(msg):
        positions.extend(position_extractor(msg) if position_extractor else _extract_positions(msg))

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(Any, detection_topic, cb, qos_profile_sensor_data)
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    with ExecutorContext(node, executor) as ec:
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout_sec:
                ec.spin_once(timeout_sec=0.1)
                for pos in positions:
                    d = _dist3(pos, expected_position)
                    if d <= search_radius:
                        result.detected = True
                        result.detection_count = len(positions)
                        result.closest_position = pos
                        result.details = f"Object at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), dist={d:.2f}m"
                        return result
        finally:
            node.destroy_subscription(sub)

    result.detection_count = len(positions)
    if positions:
        closest = min(positions, key=lambda p: _dist3(p, expected_position))
        result.closest_position = closest
        result.details = f"No object within {search_radius}m, closest at {_dist3(closest, expected_position):.2f}m"
    else:
        result.details = f"No detections on {detection_topic} within {timeout_sec}s"
    return result


def check_object_detected_by_class(
    node: Node, detection_topic: str, object_class: str,
    min_confidence: float = 0.5, timeout_sec: float = 10.0,
    class_extractor: Optional[Callable[[Any], List[tuple]]] = None,
    executor=None,
) -> DetectionResult:
    """Check that an object of a specific class is detected."""
    result = DetectionResult()
    detections: List[tuple] = []

    def cb(msg):
        detections.extend(class_extractor(msg) if class_extractor else _extract_class_detections(msg))

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(Any, detection_topic, cb, qos_profile_sensor_data)
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    with ExecutorContext(node, executor) as ec:
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout_sec:
                ec.spin_once(timeout_sec=0.1)
                for cls_name, conf, pos in detections:
                    if cls_name.lower() == object_class.lower() and conf >= min_confidence:
                        result.detected = True
                        result.detection_count = sum(
                            1 for c, co, _ in detections
                            if c.lower() == object_class.lower() and co >= min_confidence
                        )
                        result.closest_position = pos
                        result.details = f"Detected '{cls_name}' confidence={conf:.2f}"
                        return result
        finally:
            node.destroy_subscription(sub)

    matches = [(c, co) for c, co, _ in detections if c.lower() == object_class.lower()]
    if matches:
        result.details = f"'{object_class}' max confidence {max(co for _, co in matches):.2f} < {min_confidence}"
    elif detections:
        result.details = f"'{object_class}' not found. Classes: {set(c for c, _, _ in detections)}"
    else:
        result.details = f"No detections on {detection_topic} within {timeout_sec}s"
    return result


def check_min_objects_detected(
    node: Node, detection_topic: str, min_count: int,
    timeout_sec: float = 10.0,
    count_extractor: Optional[Callable[[Any], int]] = None,
    executor=None,
) -> DetectionResult:
    """Check that a minimum number of objects are detected."""
    result = DetectionResult()
    max_seen = [0]

    def cb(msg):
        count = count_extractor(msg) if count_extractor else _extract_count(msg)
        max_seen[0] = max(max_seen[0], count)

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(Any, detection_topic, cb, qos_profile_sensor_data)
    except Exception as e:
        result.details = f"Failed to subscribe to {detection_topic}: {e}"
        return result

    with ExecutorContext(node, executor) as ec:
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < timeout_sec:
                ec.spin_once(timeout_sec=0.1)
                if max_seen[0] >= min_count:
                    result.detected = True
                    result.detection_count = max_seen[0]
                    result.details = f"Detected {max_seen[0]} objects (>= {min_count})"
                    return result
        finally:
            node.destroy_subscription(sub)

    result.detection_count = max_seen[0]
    result.details = f"Only {max_seen[0]} objects, need >= {min_count}"
    return result


def check_region_clear(
    node: Node, detection_topic: str, center: Point, radius: float,
    observation_period_sec: float = 5.0,
    position_extractor: Optional[Callable[[Any], List[Point]]] = None,
    executor=None,
) -> bool:
    """Check that a region is clear of detections (safety zone)."""
    found = [False]

    def cb(msg):
        if found[0]:
            return
        for pos in (position_extractor(msg) if position_extractor else _extract_positions(msg)):
            if _dist3(pos, center) <= radius:
                found[0] = True
                return

    try:
        from rclpy.qos import qos_profile_sensor_data
        sub = node.create_subscription(Any, detection_topic, cb, qos_profile_sensor_data)
    except Exception as e:
        node.get_logger().error(
            f"check_region_clear: failed to subscribe to {detection_topic}: {e}. "
            "Returning NOT clear (fail-safe).")
        return False

    with ExecutorContext(node, executor) as ec:
        try:
            t0 = time.monotonic()
            while time.monotonic() - t0 < observation_period_sec:
                ec.spin_once(timeout_sec=0.1)
                if found[0]:
                    break
        finally:
            node.destroy_subscription(sub)
    return not found[0]


# -- Generic message extractors ────────────────────────────────────────────


def _extract_positions(msg: Any) -> List[Point]:
    positions = []
    if hasattr(msg, 'detections'):
        for det in msg.detections:
            if hasattr(det, 'results') and det.results:
                if hasattr(det, 'bbox') and hasattr(det.bbox, 'center'):
                    c = det.bbox.center
                    if hasattr(c, 'position'):
                        positions.append(c.position)
            elif hasattr(det, 'position'):
                positions.append(det.position)
    if hasattr(msg, 'poses'):
        for pose in msg.poses:
            if hasattr(pose, 'position'):
                positions.append(pose.position)
    if hasattr(msg, 'points'):
        for pt in msg.points[:100]:
            if isinstance(pt, Point):
                positions.append(pt)
            elif hasattr(pt, 'x') and hasattr(pt, 'y') and hasattr(pt, 'z'):
                p = Point()
                p.x, p.y, p.z = float(pt.x), float(pt.y), float(pt.z)
                positions.append(p)
    return positions


def _extract_class_detections(msg: Any) -> List[tuple]:
    detections = []
    if hasattr(msg, 'detections'):
        for det in msg.detections:
            cls_name, conf, pos = "", 0.0, None
            if hasattr(det, 'results') and det.results:
                hyp = getattr(det.results[0], 'hypothesis', None)
                if hyp:
                    cls_name = str(getattr(hyp, 'class_id', ''))
                    conf = float(getattr(hyp, 'score', 0.0))
            if hasattr(det, 'bbox') and hasattr(det.bbox, 'center'):
                c = det.bbox.center
                pos = getattr(c, 'position', None)
                if pos is None and hasattr(c, 'x'):
                    pos = Point()
                    pos.x, pos.y, pos.z = float(c.x), float(getattr(c, 'y', 0)), float(getattr(c, 'z', 0))
            if cls_name:
                detections.append((cls_name, conf, pos))
    return detections


def _extract_count(msg: Any) -> int:
    for attr in ('detections', 'objects', 'markers', 'poses'):
        if hasattr(msg, attr):
            return len(getattr(msg, attr))
    return 0


# -- Backward compatibility aliases (deprecated) ──────────────────────────

assert_object_detected = check_object_detected
assert_object_detected_by_class = check_object_detected_by_class
assert_min_objects_detected = check_min_objects_detected
assert_region_clear = check_region_clear
