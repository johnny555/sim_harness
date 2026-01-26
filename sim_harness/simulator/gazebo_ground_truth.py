# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Gazebo ground truth pose retrieval.

Provides direct access to entity poses from Gazebo simulation,
bypassing ROS 2 topics for ground truth validation.

This allows tests to:
1. Verify the robot actually moved in simulation (ground truth)
2. Compare odometry against ground truth to validate odom accuracy
3. Get precise positions without sensor noise or drift

Usage:
    from sim_harness.simulator.gazebo_ground_truth import GazeboGroundTruth

    with GazeboGroundTruth(world_name="empty") as gz:
        # Get model pose
        pose = gz.get_model_pose("my_robot")
        print(f"Robot at: {pose.position}")

        # Compare with odom
        odom_pos = get_odom_position()  # from ROS topic
        error = gz.compute_position_error(odom_pos, pose.position)
        assert error < 0.1, f"Odom drift too high: {error}m"
"""

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple, Callable

try:
    import gz.transport13 as transport
    from gz.msgs10 import pose_v_pb2, pose_pb2
    GZ_TRANSPORT_AVAILABLE = True
except ImportError:
    GZ_TRANSPORT_AVAILABLE = False


@dataclass
class Pose3D:
    """3D pose with position and orientation."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    @property
    def position(self) -> Tuple[float, float, float]:
        """Return position as tuple."""
        return (self.x, self.y, self.z)

    @property
    def orientation(self) -> Tuple[float, float, float]:
        """Return orientation (roll, pitch, yaw) as tuple."""
        return (self.roll, self.pitch, self.yaw)

    def distance_to(self, other: 'Pose3D') -> float:
        """Calculate 3D distance to another pose."""
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2 +
            (self.z - other.z) ** 2
        )

    def distance_2d_to(self, other: 'Pose3D') -> float:
        """Calculate 2D (x, y) distance to another pose."""
        return math.sqrt(
            (self.x - other.x) ** 2 +
            (self.y - other.y) ** 2
        )


@dataclass
class GroundTruthResult:
    """Result of a ground truth comparison."""

    success: bool
    """Whether the comparison passed."""

    ground_truth_pose: Optional[Pose3D]
    """Ground truth pose from Gazebo."""

    measured_pose: Optional[Pose3D]
    """Measured pose (e.g., from odometry)."""

    position_error: float = 0.0
    """Position error in meters."""

    orientation_error: float = 0.0
    """Orientation error in radians."""

    details: str = ""
    """Human-readable details."""


def _quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)


class GazeboGroundTruth:
    """
    Ground truth pose retrieval from Gazebo.

    Uses gz-transport to directly subscribe to Gazebo's pose topic,
    providing ground truth positions without going through ROS 2.

    Example:
        with GazeboGroundTruth(world_name="empty") as gz:
            pose = gz.get_model_pose("robot")
            print(f"Robot position: {pose.position}")
    """

    def __init__(
        self,
        world_name: str = "empty",
        pose_topic: Optional[str] = None,
        timeout_sec: float = 5.0
    ):
        """
        Initialize Gazebo ground truth interface.

        Args:
            world_name: Name of the Gazebo world (used to construct topic name)
            pose_topic: Override the pose topic (default: /world/{world_name}/pose/info)
            timeout_sec: Timeout for waiting for pose updates
        """
        if not GZ_TRANSPORT_AVAILABLE:
            raise ImportError(
                "gz-transport13 Python bindings not available. "
                "Install with: apt install python3-gz-transport13"
            )

        self.world_name = world_name
        self.pose_topic = pose_topic or f"/world/{world_name}/pose/info"
        self.timeout_sec = timeout_sec

        self._node: Optional[transport.Node] = None
        self._poses: Dict[str, Pose3D] = {}
        self._poses_lock = threading.Lock()
        self._last_update_time: float = 0.0
        self._running = False
        self._spin_thread: Optional[threading.Thread] = None

    def __enter__(self) -> 'GazeboGroundTruth':
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Context manager exit."""
        self.stop()

    def start(self) -> bool:
        """
        Start listening for pose updates.

        Returns:
            True if successfully subscribed to pose topic.
        """
        if self._running:
            return True

        self._node = transport.Node()
        self._running = True

        # Subscribe to pose topic
        subscribed = self._node.subscribe(
            pose_v_pb2.Pose_V,
            self.pose_topic,
            self._pose_callback
        )

        if not subscribed:
            self._running = False
            return False

        # Start spin thread
        self._spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin_thread.start()

        # Wait for first update
        start = time.monotonic()
        while time.monotonic() - start < self.timeout_sec:
            if self._last_update_time > 0:
                return True
            time.sleep(0.1)

        return self._last_update_time > 0

    def stop(self) -> None:
        """Stop listening for pose updates."""
        self._running = False
        if self._spin_thread:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        self._node = None

    def _spin_loop(self) -> None:
        """Background thread to process callbacks."""
        while self._running:
            time.sleep(0.01)

    def _pose_callback(self, msg: pose_v_pb2.Pose_V, info) -> None:
        """Handle incoming pose messages."""
        with self._poses_lock:
            for pose in msg.pose:
                name = pose.name
                if not name:
                    continue

                # Convert quaternion to Euler
                roll, pitch, yaw = _quaternion_to_euler(
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                )

                self._poses[name] = Pose3D(
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z,
                    roll=roll,
                    pitch=pitch,
                    yaw=yaw
                )

            self._last_update_time = time.monotonic()

    def get_model_pose(self, model_name: str, timeout_sec: Optional[float] = None) -> Optional[Pose3D]:
        """
        Get the current pose of a model.

        Args:
            model_name: Name of the model in Gazebo
            timeout_sec: How long to wait for pose. If None, uses self.timeout_sec.
                Note: A value of 0.0 means no waiting (immediate return if not found).

        Returns:
            Pose3D if found, None otherwise
        """
        timeout = self.timeout_sec if timeout_sec is None else timeout_sec
        start = time.monotonic()

        while time.monotonic() - start < timeout:
            with self._poses_lock:
                if model_name in self._poses:
                    return self._poses[model_name]
            time.sleep(0.05)

        return None

    def get_all_poses(self) -> Dict[str, Pose3D]:
        """
        Get poses of all known models.

        Returns:
            Dictionary mapping model names to poses.
        """
        with self._poses_lock:
            return dict(self._poses)

    def list_models(self) -> list:
        """
        List all known model names.

        Returns:
            List of model names.
        """
        with self._poses_lock:
            return list(self._poses.keys())

    def wait_for_model(self, model_name: str, timeout_sec: Optional[float] = None) -> bool:
        """
        Wait for a model to appear in the simulation.

        Args:
            model_name: Name of the model to wait for
            timeout_sec: How long to wait

        Returns:
            True if model found, False on timeout.
        """
        return self.get_model_pose(model_name, timeout_sec) is not None

    def compute_position_error(
        self,
        measured: Tuple[float, float, float],
        ground_truth: Tuple[float, float, float]
    ) -> float:
        """
        Compute 3D position error between measured and ground truth.

        Args:
            measured: Measured position (x, y, z)
            ground_truth: Ground truth position (x, y, z)

        Returns:
            Euclidean distance error in meters.
        """
        return math.sqrt(
            (measured[0] - ground_truth[0]) ** 2 +
            (measured[1] - ground_truth[1]) ** 2 +
            (measured[2] - ground_truth[2]) ** 2
        )

    def compare_odom_to_ground_truth(
        self,
        model_name: str,
        odom_position: Tuple[float, float, float],
        odom_orientation: Optional[Tuple[float, float, float, float]] = None,
        position_tolerance: float = 0.5,
        yaw_tolerance: float = 0.1
    ) -> GroundTruthResult:
        """
        Compare odometry reading to ground truth.

        Args:
            model_name: Name of the model in Gazebo
            odom_position: Position from odometry (x, y, z)
            odom_orientation: Quaternion from odometry (x, y, z, w). If provided,
                yaw is also compared against ground truth. If None, only position
                is compared.
            position_tolerance: Maximum acceptable position error (meters)
            yaw_tolerance: Maximum acceptable yaw error (radians). Only used
                if odom_orientation is provided.

        Returns:
            GroundTruthResult with:
            - success: True if position error <= position_tolerance AND
              (if orientation provided) yaw error <= yaw_tolerance
            - position_error: 3D Euclidean distance between odom and ground truth
            - orientation_error: Yaw error in radians (0.0 if orientation not provided)
        """
        gt_pose = self.get_model_pose(model_name)

        if gt_pose is None:
            return GroundTruthResult(
                success=False,
                ground_truth_pose=None,
                measured_pose=None,
                details=f"Model '{model_name}' not found in Gazebo"
            )

        # Create measured pose
        measured_pose = Pose3D(
            x=odom_position[0],
            y=odom_position[1],
            z=odom_position[2]
        )

        if odom_orientation:
            roll, pitch, yaw = _quaternion_to_euler(*odom_orientation)
            measured_pose.roll = roll
            measured_pose.pitch = pitch
            measured_pose.yaw = yaw

        # Compute errors
        position_error = gt_pose.distance_to(measured_pose)

        yaw_error = 0.0
        if odom_orientation:
            yaw_error = abs(gt_pose.yaw - measured_pose.yaw)
            # Normalize to [-pi, pi]
            while yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            yaw_error = abs(yaw_error)

        success = position_error <= position_tolerance
        if odom_orientation:
            success = success and yaw_error <= yaw_tolerance

        details = (
            f"Position error: {position_error:.3f}m "
            f"(tolerance: {position_tolerance}m)"
        )
        if odom_orientation:
            details += f", Yaw error: {math.degrees(yaw_error):.1f}deg"

        return GroundTruthResult(
            success=success,
            ground_truth_pose=gt_pose,
            measured_pose=measured_pose,
            position_error=position_error,
            orientation_error=yaw_error,
            details=details
        )


def get_model_pose_once(
    model_name: str,
    world_name: str = "empty",
    timeout_sec: float = 5.0
) -> Optional[Pose3D]:
    """
    Convenience function to get a model pose without maintaining a connection.

    Args:
        model_name: Name of the model in Gazebo
        world_name: Name of the Gazebo world
        timeout_sec: Timeout for connection and pose retrieval

    Returns:
        Pose3D if found, None otherwise.
    """
    with GazeboGroundTruth(world_name=world_name, timeout_sec=timeout_sec) as gz:
        return gz.get_model_pose(model_name)
