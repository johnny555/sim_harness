# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Robot readiness check module.

Provides a configurable checklist pattern for verifying a robot system
is ready for operation. Works on both real robots and simulations.

Usage:
    from sim_harness.core.readiness_check import ReadinessCheck, CheckResult

    # Quick check with defaults
    check = ReadinessCheck(node)
    result = check.run()
    if result.ready:
        print("Robot is ready!")

    # Custom checklist
    check = ReadinessCheck(node)
    check.add_topic("/scan", LaserScan)
    check.add_topic("/imu", Imu)
    check.add_node("controller_manager")
    check.add_transform("map", "base_link")
    result = check.run()
"""

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Type

from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class CheckStatus(Enum):
    """Status of an individual check."""
    PENDING = "pending"
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"
    TIMEOUT = "timeout"


class CheckCategory(Enum):
    """Category of check for grouping."""
    TOPIC = "topic"
    NODE = "node"
    SERVICE = "service"
    ACTION = "action"
    TRANSFORM = "transform"
    LIFECYCLE = "lifecycle"
    CONTROLLER = "controller"
    CUSTOM = "custom"


@dataclass
class CheckItem:
    """Definition of a single check to perform."""
    name: str
    category: CheckCategory
    check_fn: Callable[[], bool]
    timeout_sec: float = 5.0
    required: bool = True
    description: str = ""


@dataclass
class CheckItemResult:
    """Result of a single check."""
    name: str
    category: CheckCategory
    status: CheckStatus
    duration_sec: float = 0.0
    message: str = ""
    required: bool = True


@dataclass
class CheckResult:
    """Result of a complete readiness check."""
    ready: bool
    """Whether all required checks passed."""

    total_checks: int
    """Total number of checks performed."""

    passed: int
    """Number of checks that passed."""

    failed: int
    """Number of checks that failed."""

    skipped: int
    """Number of checks that were skipped."""

    duration_sec: float
    """Total time for all checks."""

    items: List[CheckItemResult] = field(default_factory=list)
    """Individual check results."""

    def summary(self) -> str:
        """Return a summary string."""
        status = "READY" if self.ready else "NOT READY"
        return (
            f"{status}: {self.passed}/{self.total_checks} checks passed "
            f"({self.failed} failed, {self.skipped} skipped) "
            f"in {self.duration_sec:.1f}s"
        )

    def failed_checks(self) -> List[CheckItemResult]:
        """Return list of failed checks."""
        return [item for item in self.items if item.status == CheckStatus.FAILED]

    def by_category(self, category: CheckCategory) -> List[CheckItemResult]:
        """Return checks for a specific category."""
        return [item for item in self.items if item.category == category]


class ReadinessCheck:
    """
    Configurable readiness check for robot systems.

    Provides a fluent API for building a checklist of items to verify,
    then runs all checks and reports results.

    Example:
        check = ReadinessCheck(node)
        check.add_topic("/scan", LaserScan, required=True)
        check.add_topic("/camera/image", Image, required=False)
        check.add_node("robot_state_publisher")
        check.add_transform("odom", "base_link")

        result = check.run()
        if not result.ready:
            for item in result.failed_checks():
                print(f"FAILED: {item.name} - {item.message}")
    """

    def __init__(
        self,
        node: Node,
        default_timeout: float = 5.0,
        stop_on_required_failure: bool = False
    ):
        """
        Initialize readiness check.

        Args:
            node: ROS 2 node for subscriptions/service calls
            default_timeout: Default timeout for each check
            stop_on_required_failure: Stop checking if a required check fails
        """
        self.node = node
        self.default_timeout = default_timeout
        self.stop_on_required_failure = stop_on_required_failure
        self._checks: List[CheckItem] = []
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)

    def add_topic(
        self,
        topic: str,
        msg_type: Type,
        timeout_sec: Optional[float] = None,
        required: bool = True,
        min_messages: int = 1,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a topic publishing check.

        Args:
            topic: Topic name
            msg_type: Message type class
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            min_messages: Minimum messages to receive
            description: Human-readable description

        Returns:
            self for chaining
        """
        timeout = timeout_sec or self.default_timeout

        def check_fn() -> bool:
            received = []

            def callback(msg):
                received.append(msg)

            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE
            )
            sub = self.node.create_subscription(msg_type, topic, callback, qos)

            try:
                start = time.monotonic()
                while len(received) < min_messages and time.monotonic() - start < timeout:
                    self._executor.spin_once(timeout_sec=0.05)
                return len(received) >= min_messages
            finally:
                self.node.destroy_subscription(sub)

        self._checks.append(CheckItem(
            name=f"topic:{topic}",
            category=CheckCategory.TOPIC,
            check_fn=check_fn,
            timeout_sec=timeout,
            required=required,
            description=description or f"Topic {topic} publishing {msg_type.__name__}"
        ))
        return self

    def add_node(
        self,
        node_name: str,
        timeout_sec: Optional[float] = None,
        required: bool = True,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a node running check.

        Args:
            node_name: Name of the node to check
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            description: Human-readable description

        Returns:
            self for chaining
        """
        timeout = timeout_sec or self.default_timeout

        def check_fn() -> bool:
            start = time.monotonic()
            while time.monotonic() - start < timeout:
                node_names = self.node.get_node_names()
                if node_name in node_names:
                    return True
                # Also check with namespace variations
                for name in node_names:
                    if name.endswith('/' + node_name) or name == node_name:
                        return True
                self._executor.spin_once(timeout_sec=0.1)
            return False

        self._checks.append(CheckItem(
            name=f"node:{node_name}",
            category=CheckCategory.NODE,
            check_fn=check_fn,
            timeout_sec=timeout,
            required=required,
            description=description or f"Node {node_name} running"
        ))
        return self

    def add_service(
        self,
        service_name: str,
        timeout_sec: Optional[float] = None,
        required: bool = True,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a service availability check.

        Checks whether the specified service appears in the node's
        service list within the timeout period.

        Args:
            service_name: Name of the service (can be full path or just name)
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            description: Human-readable description

        Returns:
            self for chaining
        """
        timeout = timeout_sec or self.default_timeout

        def check_fn() -> bool:
            start = time.monotonic()
            while time.monotonic() - start < timeout:
                services = self.node.get_service_names_and_types()
                for name, _ in services:
                    if name == service_name or name.endswith('/' + service_name):
                        return True
                self._executor.spin_once(timeout_sec=0.1)
            return False

        self._checks.append(CheckItem(
            name=f"service:{service_name}",
            category=CheckCategory.SERVICE,
            check_fn=check_fn,
            timeout_sec=timeout,
            required=required,
            description=description or f"Service {service_name} available"
        ))
        return self

    def add_transform(
        self,
        parent_frame: str,
        child_frame: str,
        timeout_sec: Optional[float] = None,
        required: bool = True,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a TF transform availability check.

        Args:
            parent_frame: Parent frame ID
            child_frame: Child frame ID
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            description: Human-readable description

        Returns:
            self for chaining
        """
        timeout = timeout_sec or self.default_timeout

        def check_fn() -> bool:
            try:
                from tf2_ros import Buffer, TransformListener
                from rclpy.duration import Duration

                tf_buffer = Buffer()
                tf_listener = TransformListener(tf_buffer, self.node)

                start = time.monotonic()
                while time.monotonic() - start < timeout:
                    try:
                        tf_buffer.lookup_transform(
                            parent_frame, child_frame,
                            self.node.get_clock().now(),
                            timeout=Duration(seconds=0.1)
                        )
                        return True
                    except Exception:
                        self._executor.spin_once(timeout_sec=0.1)

                return False
            except ImportError:
                # tf2_ros not available
                return False

        self._checks.append(CheckItem(
            name=f"tf:{parent_frame}->{child_frame}",
            category=CheckCategory.TRANSFORM,
            check_fn=check_fn,
            timeout_sec=timeout,
            required=required,
            description=description or f"Transform {parent_frame} -> {child_frame}"
        ))
        return self

    def add_lifecycle_node(
        self,
        node_name: str,
        expected_state: str = "active",
        timeout_sec: Optional[float] = None,
        required: bool = True,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a lifecycle node state check.

        Args:
            node_name: Name of the lifecycle node
            expected_state: Expected state (active, inactive, etc.)
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            description: Human-readable description

        Returns:
            self for chaining
        """
        timeout = timeout_sec or self.default_timeout

        def check_fn() -> bool:
            try:
                from lifecycle_msgs.srv import GetState

                service_name = f"/{node_name}/get_state"
                client = self.node.create_client(GetState, service_name)

                try:
                    if not client.wait_for_service(timeout_sec=timeout / 2):
                        return False

                    request = GetState.Request()
                    future = client.call_async(request)

                    start = time.monotonic()
                    while not future.done() and time.monotonic() - start < timeout / 2:
                        self._executor.spin_once(timeout_sec=0.05)

                    if future.done():
                        result = future.result()
                        return result.current_state.label.lower() == expected_state.lower()
                    return False
                finally:
                    self.node.destroy_client(client)
            except Exception:
                return False

        self._checks.append(CheckItem(
            name=f"lifecycle:{node_name}",
            category=CheckCategory.LIFECYCLE,
            check_fn=check_fn,
            timeout_sec=timeout,
            required=required,
            description=description or f"Lifecycle node {node_name} is {expected_state}"
        ))
        return self

    def add_controller(
        self,
        controller_name: str,
        expected_state: str = "active",
        controller_manager: str = "controller_manager",
        timeout_sec: Optional[float] = None,
        required: bool = True,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a ros2_control controller state check.

        Args:
            controller_name: Name of the controller
            expected_state: Expected state (active, inactive, etc.)
            controller_manager: Controller manager node name
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            description: Human-readable description

        Returns:
            self for chaining
        """
        timeout = timeout_sec or self.default_timeout

        def check_fn() -> bool:
            try:
                from controller_manager_msgs.srv import ListControllers

                service_name = f"/{controller_manager}/list_controllers"
                client = self.node.create_client(ListControllers, service_name)

                try:
                    if not client.wait_for_service(timeout_sec=timeout / 2):
                        return False

                    request = ListControllers.Request()
                    future = client.call_async(request)

                    start = time.monotonic()
                    while not future.done() and time.monotonic() - start < timeout / 2:
                        self._executor.spin_once(timeout_sec=0.05)

                    if future.done():
                        result = future.result()
                        for controller in result.controller:
                            if controller.name == controller_name:
                                return controller.state.lower() == expected_state.lower()
                    return False
                finally:
                    self.node.destroy_client(client)
            except Exception:
                return False

        self._checks.append(CheckItem(
            name=f"controller:{controller_name}",
            category=CheckCategory.CONTROLLER,
            check_fn=check_fn,
            timeout_sec=timeout,
            required=required,
            description=description or f"Controller {controller_name} is {expected_state}"
        ))
        return self

    def add_custom(
        self,
        name: str,
        check_fn: Callable[[], bool],
        timeout_sec: Optional[float] = None,
        required: bool = True,
        description: str = ""
    ) -> 'ReadinessCheck':
        """
        Add a custom check function.

        Args:
            name: Name for this check
            check_fn: Function that returns True if check passes
            timeout_sec: Timeout for this check
            required: Whether this check must pass
            description: Human-readable description

        Returns:
            self for chaining
        """
        self._checks.append(CheckItem(
            name=f"custom:{name}",
            category=CheckCategory.CUSTOM,
            check_fn=check_fn,
            timeout_sec=timeout_sec or self.default_timeout,
            required=required,
            description=description or name
        ))
        return self

    def clear(self) -> 'ReadinessCheck':
        """Clear all checks."""
        self._checks.clear()
        return self

    def run(self, verbose: bool = False) -> CheckResult:
        """
        Run all checks and return results.

        Args:
            verbose: Print progress during checks

        Returns:
            CheckResult with all check outcomes
        """
        results = []
        total_start = time.monotonic()

        for check in self._checks:
            if verbose:
                print(f"Checking {check.name}...", end=" ", flush=True)

            start = time.monotonic()
            try:
                passed = check.check_fn()
                duration = time.monotonic() - start

                status = CheckStatus.PASSED if passed else CheckStatus.FAILED
                message = "OK" if passed else "Failed"

            except Exception as e:
                duration = time.monotonic() - start
                status = CheckStatus.FAILED
                message = str(e)

            result = CheckItemResult(
                name=check.name,
                category=check.category,
                status=status,
                duration_sec=duration,
                message=message,
                required=check.required
            )
            results.append(result)

            if verbose:
                status_str = "PASS" if status == CheckStatus.PASSED else "FAIL"
                print(f"{status_str} ({duration:.1f}s)")

            # Stop early if required check failed
            if (self.stop_on_required_failure and
                check.required and
                status != CheckStatus.PASSED):
                break

        total_duration = time.monotonic() - total_start

        passed = sum(1 for r in results if r.status == CheckStatus.PASSED)
        failed = sum(1 for r in results if r.status == CheckStatus.FAILED)
        skipped = sum(1 for r in results if r.status == CheckStatus.SKIPPED)

        # Ready if all required checks passed
        required_failed = sum(
            1 for r in results
            if r.required and r.status != CheckStatus.PASSED
        )
        ready = required_failed == 0

        return CheckResult(
            ready=ready,
            total_checks=len(results),
            passed=passed,
            failed=failed,
            skipped=skipped,
            duration_sec=total_duration,
            items=results
        )


def create_standard_check(
    node: Node,
    sensors: Optional[Dict[str, Type]] = None,
    nodes: Optional[List[str]] = None,
    controllers: Optional[List[str]] = None,
    transforms: Optional[List[tuple]] = None,
    lifecycle_nodes: Optional[List[str]] = None,
    timeout: float = 5.0
) -> ReadinessCheck:
    """
    Create a readiness check with common configurations.

    Args:
        node: ROS 2 node
        sensors: Dict of topic -> message type for sensor checks
        nodes: List of node names to check
        controllers: List of controller names to check
        transforms: List of (parent, child) frame tuples
        lifecycle_nodes: List of lifecycle node names (checks for active state)
        timeout: Default timeout for checks

    Returns:
        Configured ReadinessCheck instance

    Example:
        from sensor_msgs.msg import LaserScan, Imu

        check = create_standard_check(
            node,
            sensors={"/scan": LaserScan, "/imu": Imu},
            nodes=["robot_state_publisher"],
            controllers=["joint_state_broadcaster"],
            transforms=[("odom", "base_link"), ("map", "odom")],
            lifecycle_nodes=["controller_manager"]
        )
        result = check.run()
    """
    check = ReadinessCheck(node, default_timeout=timeout)

    if sensors:
        for topic, msg_type in sensors.items():
            check.add_topic(topic, msg_type)

    if nodes:
        for node_name in nodes:
            check.add_node(node_name)

    if controllers:
        for controller in controllers:
            check.add_controller(controller)

    if transforms:
        for parent, child in transforms:
            check.add_transform(parent, child)

    if lifecycle_nodes:
        for lc_node in lifecycle_nodes:
            check.add_lifecycle_node(lc_node)

    return check
