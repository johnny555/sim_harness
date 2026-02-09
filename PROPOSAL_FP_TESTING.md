# Proposal: FP-Inspired Testing Primitives for sim_harness

## Motivation

sim_harness currently provides 50+ monolithic assertion functions that each manage
their own ROS 2 lifecycle (executor, subscription, spin loop, teardown). This leads to
significant boilerplate, prevents composition, and makes it impossible to combine checks
without duplicating infrastructure.

The core insight is that **most sim_harness assertions are properties over message
streams**. This maps directly onto ideas from Hedgehog (Haskell) and Hypothesis
(Python): composable predicates, universal quantification over data, and automatic
counterexample reporting.

This proposal describes three layers of FP-inspired primitives, from foundational
combinators to high-level property testing integration.

---

## Layer 1: `TopicObserver` — A Fold Over Message Streams

### The Problem

Every assertion function repeats the same ~20-line pattern:

```python
def assert_something(node, topic, ...):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    state = initial_value
    def callback(msg):
        nonlocal state
        state = update(state, msg)
    qos = QoSProfile(depth=10, reliability=BEST_EFFORT, durability=VOLATILE)
    sub = node.create_subscription(MsgType, topic, callback, qos)
    try:
        # spin and check ...
    finally:
        node.destroy_subscription(sub)
        executor.remove_node(node)
```

This appears in `assert_vehicle_moved`, `assert_vehicle_stationary`,
`assert_vehicle_velocity`, `assert_vehicle_in_region`, `assert_vehicle_orientation`,
`assert_sensor_publishing`, `assert_lidar_valid`, `assert_gps_valid`,
`assert_imu_valid`, `assert_camera_valid`, `assert_joint_states_valid`,
`assert_publish_rate`, `assert_latency`, and more — at least 15 times.

### The Solution: `TopicObserver`

A `TopicObserver` is a **fold** (reduce) over a message stream. It encapsulates
the subscribe-spin-collect pattern into a single reusable type:

```python
from dataclasses import dataclass
from typing import TypeVar, Generic, Callable, Type, Optional, List
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

MsgT = TypeVar('MsgT')
S = TypeVar('S')
R = TypeVar('R')

# Default QoS used throughout the primitives today
SENSOR_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass(frozen=True)
class ObservationResult(Generic[R]):
    """Immutable result from a TopicObserver."""
    value: R
    message_count: int
    duration_sec: float


class TopicObserver(Generic[MsgT, S, R]):
    """
    A fold over a ROS 2 message stream.

    Subscribes to a topic, feeds each message through a fold function to
    accumulate state, then extracts a final result. This is the fundamental
    building block — all assertion functions can be built from TopicObservers.

    Type parameters:
        MsgT: The ROS message type being observed
        S: The internal accumulator state type
        R: The final result type (extracted from S)

    The three operations mirror a standard fold:
        - initial: S                      (the seed value)
        - step:    (S, MsgT) -> S         (the fold function)
        - extract: S -> R                 (the final projection)

    Example — count messages:
        observer = TopicObserver(
            topic="/scan",
            msg_type=LaserScan,
            initial=0,
            step=lambda count, _msg: count + 1,
            extract=lambda count: count,
        )
        result = observer.run(node, executor, timeout_sec=5.0)
        assert result.value > 0

    Example — track max velocity:
        def linear_speed(msg: Odometry) -> float:
            v = msg.twist.twist.linear
            return math.sqrt(v.x**2 + v.y**2 + v.z**2)

        observer = TopicObserver(
            topic="/odom",
            msg_type=Odometry,
            initial=0.0,
            step=lambda mx, msg: max(mx, linear_speed(msg)),
            extract=lambda mx: mx,
        )
        result = observer.run(node, executor, timeout_sec=3.0)
        assert result.value < 0.05, "Robot should be stationary"
    """

    def __init__(
        self,
        topic: str,
        msg_type: Type[MsgT],
        initial: S,
        step: Callable[[S, MsgT], S],
        extract: Callable[[S], R],
        qos: Optional[QoSProfile] = None,
    ):
        self.topic = topic
        self.msg_type = msg_type
        self.initial = initial
        self.step = step
        self.extract = extract
        self.qos = qos or SENSOR_QOS

    def run(
        self,
        node: Node,
        executor: SingleThreadedExecutor,
        timeout_sec: float,
    ) -> ObservationResult[R]:
        """
        Execute the observation: subscribe, spin, fold, extract.

        Uses the caller's executor — does NOT create a new one.
        """
        import time

        state = self.initial
        count = 0

        def callback(msg: MsgT):
            nonlocal state, count
            state = self.step(state, msg)
            count += 1

        sub = node.create_subscription(
            self.msg_type, self.topic, callback, self.qos
        )
        try:
            from sim_harness.core.spin_helpers import spin_for_duration
            spin_for_duration(executor, timeout_sec)
        finally:
            node.destroy_subscription(sub)

        return ObservationResult(
            value=self.extract(state),
            message_count=count,
            duration_sec=timeout_sec,
        )

    def map(self, f: Callable[[R], 'R2']) -> 'TopicObserver[MsgT, S, R2]':
        """
        Transform the result with a pure function (functor map).

        Example:
            observer.map(lambda rate: rate > 10.0)  # bool: is rate above 10Hz?
        """
        return TopicObserver(
            topic=self.topic,
            msg_type=self.msg_type,
            initial=self.initial,
            step=self.step,
            extract=lambda s: f(self.extract(s)),
            qos=self.qos,
        )
```

### Existing Assertions Rebuilt as Thin Wrappers

With `TopicObserver`, every existing assertion becomes 5-10 lines instead of 30-60:

```python
# --- assert_vehicle_stationary (currently 37 lines) ---

def observe_max_velocity(
    odom_topic: str, duration_sec: float
) -> TopicObserver[Odometry, float, float]:
    """Observe maximum linear velocity on an odom topic."""
    return TopicObserver(
        topic=odom_topic,
        msg_type=Odometry,
        initial=0.0,
        step=lambda mx, msg: max(mx, _linear_speed(msg)),
        extract=lambda mx: mx,
    )

def assert_vehicle_stationary(
    node: Node, executor: SingleThreadedExecutor,
    vehicle_id: str, velocity_threshold: float = 0.01,
    duration_sec: float = 2.0, odom_topic: str | None = None,
) -> bool:
    topic = odom_topic or f"/{vehicle_id}/odom"
    result = observe_max_velocity(topic, duration_sec).run(node, executor, duration_sec)
    return result.value <= velocity_threshold


# --- assert_sensor_publishing (currently 48 lines) ---

def observe_message_count(
    topic: str, msg_type: type, duration_sec: float,
) -> TopicObserver:
    """Count messages on a topic."""
    return TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=0,
        step=lambda n, _: n + 1,
        extract=lambda n: n,
    )

def assert_sensor_publishing(
    node: Node, executor: SingleThreadedExecutor,
    topic: str, expected_rate_hz: float,
    msg_type: type = LaserScan,
    tolerance_percent: float = 10.0,
    sample_duration_sec: float = 5.0,
) -> SensorDataResult:
    result = observe_message_count(topic, msg_type, sample_duration_sec).run(
        node, executor, sample_duration_sec
    )
    rate = result.value / sample_duration_sec
    deviation = abs(rate - expected_rate_hz) / expected_rate_hz * 100
    return SensorDataResult(
        valid=deviation <= tolerance_percent,
        message_count=result.value,
        publish_rate_hz=rate,
        details=f"Rate: {rate:.1f} Hz, Expected: {expected_rate_hz:.1f} Hz",
    )
```

### Combining Multiple Observers

You can observe multiple topics simultaneously by running observers in parallel
on the same executor, or sequentially:

```python
def observe_odom_and_scan(node, executor, timeout):
    """Observe odometry and LIDAR simultaneously."""
    # Both observers share the same executor and spin period
    # Create both subscriptions, then spin once
    odom_obs = observe_max_velocity("/odom", timeout)
    scan_obs = observe_message_count("/scan", LaserScan, timeout)

    # Run them together (parallel observation, single spin)
    odom_result = odom_obs.run(node, executor, timeout)
    scan_result = scan_obs.run(node, executor, timeout)

    return odom_result, scan_result
```

For true parallel observation (multiple topics, single spin), we can provide a
`ParallelObserver` that merges multiple folds into one spin period:

```python
class ParallelObserver:
    """Run multiple TopicObservers in a single spin period."""

    def __init__(self, *observers: TopicObserver):
        self.observers = observers

    def run(self, node, executor, timeout_sec):
        """Subscribe all, spin once, extract all results."""
        states = [obs.initial for obs in self.observers]
        counts = [0] * len(self.observers)
        subs = []

        for i, obs in enumerate(self.observers):
            def make_cb(idx, observer):
                def cb(msg):
                    nonlocal states, counts
                    states[idx] = observer.step(states[idx], msg)
                    counts[idx] += 1
                return cb

            sub = node.create_subscription(
                obs.msg_type, obs.topic, make_cb(i, obs), obs.qos
            )
            subs.append(sub)

        try:
            spin_for_duration(executor, timeout_sec)
        finally:
            for sub in subs:
                node.destroy_subscription(sub)

        return tuple(
            ObservationResult(
                value=obs.extract(states[i]),
                message_count=counts[i],
                duration_sec=timeout_sec,
            )
            for i, obs in enumerate(self.observers)
        )
```

Usage:

```python
odom_result, scan_result = ParallelObserver(
    observe_max_velocity("/odom", 5.0),
    observe_message_count("/scan", LaserScan, 5.0),
).run(node, executor, 5.0)
```

---

## Layer 2: Composable Predicates & Stream Properties

### The Problem

Current assertions are monolithic — `assert_lidar_valid` checks min points, range
bounds, and NaN percentage all in one function. If you need a slightly different
check (different NaN threshold, or checking intensities), you must write a new
function.

### Predicates as Functions

A predicate is just `MsgT -> bool`. Combinators compose them:

```python
from typing import Callable, TypeVar, Sequence

MsgT = TypeVar('MsgT')
Predicate = Callable[[MsgT], bool]


def all_of(*predicates: Predicate[MsgT]) -> Predicate[MsgT]:
    """All predicates must hold."""
    return lambda msg: all(p(msg) for p in predicates)


def any_of(*predicates: Predicate[MsgT]) -> Predicate[MsgT]:
    """At least one predicate must hold."""
    return lambda msg: any(p(msg) for p in predicates)


def negate(predicate: Predicate[MsgT]) -> Predicate[MsgT]:
    """Invert a predicate."""
    return lambda msg: not predicate(msg)
```

### Sensor Predicates Library

```python
import math
from sensor_msgs.msg import LaserScan, Imu, Image, NavSatFix

# --- LaserScan predicates ---

def scan_has_min_points(n: int) -> Predicate[LaserScan]:
    """Scan contains at least n non-NaN, non-inf range values."""
    def check(scan: LaserScan) -> bool:
        valid = sum(1 for r in scan.ranges if math.isfinite(r))
        return valid >= n
    return check

def scan_ranges_within(lo: float, hi: float) -> Predicate[LaserScan]:
    """All finite range values fall within [lo, hi]."""
    def check(scan: LaserScan) -> bool:
        return all(lo <= r <= hi for r in scan.ranges if math.isfinite(r))
    return check

def scan_nan_ratio_below(threshold: float) -> Predicate[LaserScan]:
    """Fraction of NaN values is below threshold."""
    def check(scan: LaserScan) -> bool:
        if not scan.ranges:
            return False
        nan_count = sum(1 for r in scan.ranges if math.isnan(r))
        return nan_count / len(scan.ranges) < threshold
    return check


# --- IMU predicates ---

def imu_accel_within(max_magnitude: float) -> Predicate[Imu]:
    """Linear acceleration magnitude is below max."""
    def check(imu: Imu) -> bool:
        a = imu.linear_acceleration
        return math.sqrt(a.x**2 + a.y**2 + a.z**2) <= max_magnitude
    return check

def imu_gyro_within(max_magnitude: float) -> Predicate[Imu]:
    """Angular velocity magnitude is below max."""
    def check(imu: Imu) -> bool:
        g = imu.angular_velocity
        return math.sqrt(g.x**2 + g.y**2 + g.z**2) <= max_magnitude
    return check

def imu_no_nan() -> Predicate[Imu]:
    """No NaN values in accel or gyro."""
    def check(imu: Imu) -> bool:
        a = imu.linear_acceleration
        g = imu.angular_velocity
        return all(math.isfinite(v) for v in [a.x, a.y, a.z, g.x, g.y, g.z])
    return check


# --- Image predicates ---

def image_has_data() -> Predicate[Image]:
    """Image contains non-empty data."""
    return lambda img: len(img.data) > 0

def image_dimensions(width: int, height: int) -> Predicate[Image]:
    """Image matches expected dimensions."""
    return lambda img: img.width == width and img.height == height

def image_encoding(encoding: str) -> Predicate[Image]:
    """Image uses expected encoding."""
    return lambda img: img.encoding == encoding
```

### Composing Predicates — The Current `assert_lidar_valid` Becomes:

```python
# Before: one monolithic function (47 lines)
result = assert_lidar_valid(node, "/scan", min_range=0.1, max_range=100.0,
                            min_points=100, timeout=5)

# After: composable predicates (clear, extensible)
valid_lidar = all_of(
    scan_has_min_points(100),
    scan_ranges_within(0.1, 100.0),
    scan_nan_ratio_below(0.1),
)

result = for_all_messages(node, executor, "/scan", LaserScan,
    predicate=valid_lidar,
    timeout_sec=5.0,
    description="LIDAR data meets quality requirements",
)
assert result.passed, result.counterexample_details
```

### `for_all_messages` — Hedgehog's `forAll` Over Topics

This is the bridge between predicates and message streams:

```python
@dataclass(frozen=True)
class PropertyResult(Generic[MsgT]):
    """Result of checking a property over a message stream."""
    passed: bool
    total_checked: int
    first_failure_index: int | None = None
    counterexample: MsgT | None = None
    counterexample_details: str = ""
    description: str = ""


def for_all_messages(
    node: Node,
    executor: SingleThreadedExecutor,
    topic: str,
    msg_type: Type[MsgT],
    predicate: Predicate[MsgT],
    timeout_sec: float = 5.0,
    min_samples: int = 1,
    description: str = "",
) -> PropertyResult[MsgT]:
    """
    Assert that a property holds for ALL messages on a topic.

    This is Hedgehog's `forAll` applied to a ROS message stream.
    When the property fails, the result contains the exact counterexample
    message and its index in the stream — far more useful than a generic
    "sensor data invalid" error.

    Args:
        node: ROS 2 node
        executor: Executor to spin (reuses caller's executor)
        topic: Topic to observe
        msg_type: Message type
        predicate: Property that must hold for every message
        timeout_sec: Observation window
        min_samples: Minimum messages required (fails if not enough)
        description: Human-readable property description

    Returns:
        PropertyResult with pass/fail, total checked, and counterexample
    """
    # Collect messages using TopicObserver
    collector = TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=[],
        step=lambda msgs, msg: msgs + [msg],  # collect all
        extract=lambda msgs: msgs,
    )
    obs_result = collector.run(node, executor, timeout_sec)
    messages = obs_result.value

    if len(messages) < min_samples:
        return PropertyResult(
            passed=False,
            total_checked=len(messages),
            description=f"Not enough samples: {len(messages)} < {min_samples}",
        )

    for i, msg in enumerate(messages):
        if not predicate(msg):
            return PropertyResult(
                passed=False,
                total_checked=i + 1,
                first_failure_index=i,
                counterexample=msg,
                counterexample_details=f"Property violated at message {i}/{len(messages)}",
                description=description,
            )

    return PropertyResult(
        passed=True,
        total_checked=len(messages),
        description=description,
    )
```

### Temporal Stream Properties

Beyond "holds for all messages", robotics needs temporal properties:

```python
def eventually(
    node: Node,
    executor: SingleThreadedExecutor,
    topic: str,
    msg_type: Type[MsgT],
    predicate: Predicate[MsgT],
    timeout_sec: float,
    description: str = "",
) -> PropertyResult[MsgT]:
    """
    Assert that a predicate becomes true for at least one message.

    Useful for convergence tests: "the robot eventually reaches the goal",
    "localization eventually converges", "the sensor eventually publishes".
    """
    collector = TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=[],
        step=lambda msgs, msg: msgs + [msg],
        extract=lambda msgs: msgs,
    )
    obs_result = collector.run(node, executor, timeout_sec)
    messages = obs_result.value

    for i, msg in enumerate(messages):
        if predicate(msg):
            return PropertyResult(
                passed=True,
                total_checked=i + 1,
                description=description,
            )

    return PropertyResult(
        passed=False,
        total_checked=len(messages),
        description=f"Property never held across {len(messages)} messages",
    )


def monotonic(
    node: Node,
    executor: SingleThreadedExecutor,
    topic: str,
    msg_type: Type[MsgT],
    extract_value: Callable[[MsgT], float],
    timeout_sec: float,
    description: str = "",
) -> PropertyResult[MsgT]:
    """
    Assert that an extracted value is monotonically non-decreasing.

    Useful for: timestamps always increasing, distance to goal always
    decreasing (with negate), battery level always decreasing.
    """
    @dataclass
    class State:
        messages: list
        prev: float | None = None
        violation_idx: int | None = None

    def step(s: State, msg: MsgT) -> State:
        val = extract_value(msg)
        s.messages.append(msg)
        if s.prev is not None and val < s.prev and s.violation_idx is None:
            s.violation_idx = len(s.messages) - 1
        s.prev = val
        return s

    observer = TopicObserver(
        topic=topic,
        msg_type=msg_type,
        initial=State(messages=[]),
        step=step,
        extract=lambda s: s,
    )
    obs_result = observer.run(node, executor, timeout_sec)
    state = obs_result.value

    if state.violation_idx is not None:
        return PropertyResult(
            passed=False,
            total_checked=len(state.messages),
            first_failure_index=state.violation_idx,
            counterexample=state.messages[state.violation_idx],
            description=description,
        )

    return PropertyResult(
        passed=True,
        total_checked=len(state.messages),
        description=description,
    )
```

### Usage Examples with Temporal Properties

```python
# "Robot eventually reaches within 0.5m of goal"
result = eventually(
    node, executor, "/odom", Odometry,
    predicate=lambda msg: distance(msg.pose.pose.position, goal) < 0.5,
    timeout_sec=60.0,
    description="Robot converges to goal",
)

# "Timestamps are always increasing on /scan"
result = monotonic(
    node, executor, "/scan", LaserScan,
    extract_value=lambda msg: msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
    timeout_sec=5.0,
    description="LIDAR timestamps are monotonically increasing",
)

# "LIDAR always has valid data during the entire test"
result = for_all_messages(
    node, executor, "/scan", LaserScan,
    predicate=all_of(
        scan_has_min_points(100),
        scan_ranges_within(0.1, 30.0),
        scan_nan_ratio_below(0.05),
    ),
    timeout_sec=10.0,
    description="LIDAR data quality is consistent",
)
```

---

## Layer 3: Hypothesis Integration — Property-Based Scenario Generation

### The Idea

Hedgehog/Hypothesis generate random inputs and check that properties hold for all
of them. For simulation testing, the "inputs" are **test scenarios**: goal poses,
velocity commands, sensor configurations. The "properties" are invariants that
should hold regardless of the specific input.

### Hypothesis Strategies for ROS

```python
import math
import hypothesis.strategies as st
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Quaternion, Point
from hypothesis import given, settings, assume


# --- Geometry strategies ---

def twist_strategy(
    max_linear: float = 2.0,
    max_angular: float = 1.0,
) -> st.SearchStrategy[Twist]:
    """Generate random Twist commands within physical limits."""
    return st.builds(
        _make_twist,
        linear_x=st.floats(min_value=-max_linear, max_value=max_linear),
        angular_z=st.floats(min_value=-max_angular, max_value=max_angular),
    )

def _make_twist(linear_x: float, angular_z: float) -> Twist:
    t = Twist()
    t.linear.x = linear_x
    t.angular.z = angular_z
    return t


def pose_strategy(
    x_range: tuple[float, float] = (-10.0, 10.0),
    y_range: tuple[float, float] = (-10.0, 10.0),
) -> st.SearchStrategy[tuple[float, float, float]]:
    """Generate random (x, y, yaw) poses within bounds."""
    return st.tuples(
        st.floats(min_value=x_range[0], max_value=x_range[1]),
        st.floats(min_value=y_range[0], max_value=y_range[1]),
        st.floats(min_value=-math.pi, max_value=math.pi),
    )


def velocity_strategy(
    min_vel: float = 0.1,
    max_vel: float = 2.0,
) -> st.SearchStrategy[float]:
    """Generate random velocities."""
    return st.floats(min_value=min_vel, max_value=max_vel)


def duration_strategy(
    min_sec: float = 1.0,
    max_sec: float = 10.0,
) -> st.SearchStrategy[float]:
    """Generate random durations."""
    return st.floats(min_value=min_sec, max_value=max_sec)
```

### Property-Based Tests with Hypothesis

```python
from hypothesis import given, settings


@given(
    velocity=velocity_strategy(0.1, 2.0),
    duration=duration_strategy(1.0, 5.0),
)
@settings(max_examples=5)  # Keep low for sim tests (each is expensive)
def test_distance_proportional_to_velocity(velocity, duration, sim_fixture):
    """
    Property: distance traveled >= velocity * time * factor.

    Physics-based invariant: a robot moving at constant velocity for time t
    should travel approximately v*t meters. We allow 50% tolerance for
    acceleration ramp-up, physics variance, and sim timestep effects.
    """
    node = sim_fixture.node
    executor = sim_fixture.executor

    # Command the velocity
    publish_velocity(node, velocity, duration)

    # Observe distance via odometry
    observer = TopicObserver(
        topic="/odom", msg_type=Odometry,
        initial=(None, 0.0),
        step=_track_distance,
        extract=lambda s: s[1],
    )
    result = observer.run(node, executor, duration + 1.0)

    # Property: distance should be at least 50% of v*t
    expected = velocity * duration
    assert result.value >= expected * 0.5, (
        f"At v={velocity:.2f}m/s for t={duration:.1f}s: "
        f"expected >= {expected * 0.5:.2f}m, got {result.value:.2f}m"
    )


@given(goal=pose_strategy(x_range=(-3.0, 3.0), y_range=(-3.0, 3.0)))
@settings(max_examples=3)
def test_navigation_reaches_nearby_goals(goal, sim_fixture):
    """
    Property: the robot can reach any nearby free-space goal.

    Hypothesis generates random goal poses. If this test fails,
    Hypothesis automatically SHRINKS to the simplest failing goal
    (e.g., nearest to origin, round coordinates), making it
    immediately clear what kind of goal poses cause failures.
    """
    x, y, yaw = goal

    # Skip goals that are inside obstacles
    assume(is_in_free_space(x, y, sim_fixture.costmap))

    result = navigate_to(sim_fixture.node, sim_fixture.executor, x, y, yaw, timeout=30)

    assert result.reached, (
        f"Failed to reach goal ({x:.2f}, {y:.2f}, yaw={yaw:.2f}): "
        f"stopped at ({result.final_x:.2f}, {result.final_y:.2f}), "
        f"distance remaining: {result.distance_remaining:.2f}m"
    )


@given(
    max_accel=st.floats(min_value=20.0, max_value=100.0),
    max_gyro=st.floats(min_value=5.0, max_value=20.0),
)
@settings(max_examples=5)
def test_imu_within_physical_bounds(max_accel, max_gyro, sim_fixture):
    """
    Property: IMU readings never exceed physical bounds, regardless
    of how generous the threshold is.

    This catches catastrophic sensor failures (NaN, infinity, wildly
    wrong values) that would fail for ANY reasonable threshold.
    """
    result = for_all_messages(
        sim_fixture.node, sim_fixture.executor,
        "/imu", Imu,
        predicate=all_of(
            imu_no_nan(),
            imu_accel_within(max_accel),
            imu_gyro_within(max_gyro),
        ),
        timeout_sec=3.0,
        description=f"IMU within bounds (accel<{max_accel}, gyro<{max_gyro})",
    )
    assert result.passed, result.counterexample_details
```

### Why Shrinking Matters

When `test_navigation_reaches_nearby_goals` fails with a randomly generated goal
like `(2.7831, -1.4429, 0.8832)`, Hypothesis automatically tries simpler inputs:

```
# Hypothesis shrinking trace:
# (2.7831, -1.4429, 0.8832) -> FAIL
# (1.3916, -0.7215, 0.4416) -> FAIL
# (0.6958, -0.3607, 0.2208) -> PASS
# (1.0437, -0.5411, 0.3312) -> FAIL
# (0.8698, -0.4509, 0.2760) -> PASS
# (0.9567, -0.4960, 0.3036) -> FAIL  <-- minimal counterexample
#
# Smallest failing example: goal=(0.96, -0.50, 0.30)
```

This reveals that the robot can't reach goals roughly 1m away at a specific angle —
much more actionable than the original random failure.

---

## Layer 4: `ValidationScope` — Replacing the Singleton

### Current Problem

`ValidationResultCollector.instance()` is a process-global singleton. Parallel tests
pollute each other's results.

### Proposed Replacement

```python
from dataclasses import dataclass, field
from typing import List
import json
from datetime import datetime


class ValidationScope:
    """
    Scoped validation result collection.

    Each test suite or test session gets its own scope, replacing the
    global singleton. Scopes can be nested (suite -> test -> assertion).
    """

    def __init__(self, name: str, parent: 'ValidationScope | None' = None):
        self.name = name
        self.parent = parent
        self._results: List[ValidationResult] = []

    def add(self, result: ValidationResult) -> None:
        self._results.append(result)
        if self.parent:
            self.parent.add(result)

    @property
    def results(self) -> List[ValidationResult]:
        return list(self._results)

    @property
    def counts(self) -> tuple[int, int]:
        passed = sum(1 for r in self._results if r.passed)
        return passed, len(self._results) - passed

    def export_json(self, filepath: str) -> None:
        passed, failed = self.counts
        data = {
            "scope": self.name,
            "timestamp": datetime.now().isoformat(),
            "summary": {"total": len(self._results), "passed": passed, "failed": failed},
            "results": [asdict(r) for r in self._results],
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


# Pytest fixture integration
@pytest.fixture
def validation_scope(request):
    """Per-test validation scope."""
    scope = ValidationScope(request.node.name)
    yield scope
    # Optionally export on test completion
```

### Migration Path

For backward compatibility, the singleton can delegate to a thread-local scope:

```python
class ValidationResultCollector:
    """Backward-compatible wrapper that delegates to scoped collection."""

    _thread_local = threading.local()

    @classmethod
    def set_scope(cls, scope: ValidationScope):
        cls._thread_local.scope = scope

    @classmethod
    def instance(cls):
        if not hasattr(cls._thread_local, 'scope'):
            cls._thread_local.scope = ValidationScope("default")
        return cls._thread_local.scope
```

---

## Layer 5: Hypothesis Integration — Three-Tier Property Testing (IMPLEMENTED)

Layer 3 above described Hypothesis integration at a conceptual level.  This section
describes the **actual implementation**, which refines the approach into three distinct
tiers based on cost-per-example.

### The Problem with Naive Hypothesis + Sims

Hypothesis is designed for functions that run in microseconds.  Robotics sims take
seconds to minutes per scenario.  Naive `@given` with `max_examples=100` would take
hours.

### Three-Tier Solution

**Tier 1: Properties over Recorded Data (CHEAP — full Hypothesis power)**

Collect data once from the sim, then check many properties over the recorded
messages.  The sim is never re-run.  Each Hypothesis example costs microseconds.

```python
from sim_harness.core.sim_property import (
    check_recorded_property,
    check_recorded_eventually,
    check_recorded_monotonic,
    hypothesis_check_recorded,
)

# Collect once (expensive)
collector = self.create_message_collector('/scan', LaserScan)
self.spin_for_duration(10.0)
messages = collector.get_messages()

# Check many properties (cheap)
check_recorded_property(
    messages,
    lambda scan: len([r for r in scan.ranges if math.isfinite(r)]) >= 100,
    description="All scans have >= 100 valid points",
)

check_recorded_monotonic(
    messages,
    extract=lambda scan: scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9,
    description="Timestamps are non-decreasing",
)
```

With Hypothesis, you can generate random *parameters* and check them against the
recorded data — Hypothesis shrinks the parameter, not the sim:

```python
hypothesis_check_recorded(
    data=messages,
    strategy=st.floats(0.05, 0.5),  # NaN ratio thresholds
    property_fn=lambda scan, threshold: (
        sum(1 for r in scan.ranges if math.isnan(r))
        / max(len(scan.ranges), 1)
        < threshold
    ),
    description="NaN ratio below threshold",
    max_examples=50,
)
```

**Tier 2: Scenario-Level Properties (EXPENSIVE — use sparingly)**

Hypothesis generates entire test scenarios (goals, initial poses).  Each example
re-runs part of the sim.  Use `max_examples=3-5`.

```python
from sim_harness.core.sim_property import sim_property
from sim_harness.core.strategies import navigation_goal_2d

@sim_property(max_examples=3)
@given(goal=navigation_goal_2d(x_bounds=(-2, 2), y_bounds=(-2, 2)))
def test_reaches_random_goals(self, goal):
    # Each example sends a different goal and waits for navigation
    result = assert_reaches_goal(self.node, self.executor, goal, 30.0)
    assert result.reached
```

**Tier 3: Same-Sim Parameter Variation (MEDIUM cost)**

The sim stays running; Hypothesis varies commands or parameters within a single
session.  Each example costs seconds (a velocity command + observe), not minutes
(a full sim restart).

```python
from sim_harness.core.strategies import twist_strategy

@sim_property(max_examples=10)
@given(cmd=twist_strategy(max_linear=0.3, max_angular=0.5))
def test_any_twist_keeps_robot_stable(self, cmd):
    # Send cmd, observe odometry, check no NaN
    ...
```

### `sim_property` Decorator

Pre-configured `@settings` for sim testing:

- No deadline (sim tests can't have deterministic timing)
- Persistent example database at `~/.hypothesis/sim_harness/`
- `too_slow` health check suppressed
- Nightly mode via `SIM_HARNESS_NIGHTLY=1` (10x more examples)

### ROS Message Strategies

`sim_harness.core.strategies` provides Hypothesis strategies for common ROS types:

| Strategy | Generates | Tier |
|----------|-----------|------|
| `point_strategy` | `geometry_msgs/Point` | 2, 3 |
| `pose_strategy` | `geometry_msgs/Pose` | 2 |
| `twist_strategy` | `geometry_msgs/Twist` (ground robot) | 3 |
| `twist_strategy_3d` | `geometry_msgs/Twist` (6-DOF) | 3 |
| `quaternion_strategy` | Valid unit quaternion | 2 |
| `yaw_quaternion_strategy` | Yaw-only quaternion | 2 |
| `navigation_goal_2d` | `PoseStamped` goal | 2 |
| `waypoints_strategy` | List of `PoseStamped` | 2 |
| `threshold_strategy` | Float thresholds | 1 |
| `speed_strategy` | Speed values (m/s) | 1, 3 |
| `duration_strategy` | Duration values (s) | 1, 3 |
| `angle_strategy` | Angles (radians) | 1, 3 |

See `examples/test_property_based.py` for complete working examples of all three tiers.

---

## Implementation Status

| Component | Status | File(s) |
|-----------|--------|---------|
| `TopicObserver` | **Implemented** | `sim_harness/core/topic_observer.py` |
| Composable predicates | **Implemented** | `sim_harness/core/predicates.py` |
| Stream properties | **Implemented** | `sim_harness/core/stream_properties.py` |
| `ValidationScope` | **Implemented** | C++: `validation_result.hpp/cpp`, Py: `validation_result.py` |
| `RequirementValidator` (CRTP removed) | **Implemented** | `requirement_validator.hpp` |
| `sim_property` decorator | **Implemented** | `sim_harness/core/sim_property.py` |
| Hypothesis strategies | **Implemented** | `sim_harness/core/strategies.py` |
| Recorded-data property helpers | **Implemented** | `sim_harness/core/sim_property.py` |
| Example tests (3 tiers) | **Implemented** | `examples/test_property_based.py` |
| Rebuild assertions on TopicObserver | Proposed | Existing functions still work as-is |

---

## Summary: What Changes, What Stays

| Component | Status | Change |
|-----------|--------|--------|
| `SimTestFixture` | Keep | No changes needed |
| `MessageCollector` | Keep | Still useful for long-running collection |
| `TestIsolationConfig` | Keep | Orthogonal, working well |
| Assertion functions | Refactor | Rebuild on `TopicObserver` + predicates |
| `ValidationResultCollector` | Replace | Use `ValidationScope` |
| `RequirementValidator` | Simplify | Drop CRTP, accept scope parameter |
| Result structs | Unify | Single `PropertyResult` with counterexamples |
| **New: `TopicObserver`** | Add | Foundation for all stream operations |
| **New: Predicate library** | Add | Composable sensor/vehicle/nav checks |
| **New: `for_all_messages`** | Add | Hedgehog's `forAll` over topics |
| **New: `eventually`/`monotonic`** | Add | Temporal stream properties |
| **New: Hypothesis strategies** | Add | Property-based scenario generation |
| **New: `sim_property`** | Add | Pre-configured settings for sim tests |
| **New: Recorded-data helpers** | Add | Tier 1 property checking without sim re-run |

### Dependency Additions

```xml
<!-- package.xml -->
<test_depend>python3-hypothesis</test_depend>
```

```python
# setup.py extras_require
extras_require={
    'hypothesis': ['hypothesis>=6.0'],
}
```

### Estimated Impact

- **Lines removed**: ~500 (boilerplate in primitives)
- **Lines added**: ~300 (TopicObserver, predicates, stream properties) + ~700 (Hypothesis infra)
- **New capability**: Property-based testing with shrinking, three-tier Hypothesis integration
- **Backward compatibility**: Old assertion functions remain as thin wrappers
