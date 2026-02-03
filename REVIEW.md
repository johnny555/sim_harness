# sim_harness Code Review: Simplification & FP Testing Proposals

## Executive Summary

sim_harness is a well-structured dual-language (C++/Python) test harness for ROS 2
simulation testing. The core abstractions (fixture, message collector, test isolation)
are solid. However, the assertion primitives layer has significant boilerplate
duplication, the architecture relies on global mutable state, and the monolithic
assertion functions resist composition. Drawing on ideas from Hedgehog
(Haskell property-based testing) and its Python equivalent Hypothesis, this
review proposes a shift toward composable predicates, property-based testing
over message streams, and generator-driven test scenarios.

---

## Part 1: Code Review — Current Issues

### 1.1 Massive Boilerplate in Assertion Primitives

Every Python assertion function in `primitives/vehicle_assertions.py` repeats the
same ~20-line scaffolding:

```python
# This pattern appears in EVERY assertion function:
executor = SingleThreadedExecutor()
executor.add_node(node)
qos = QoSProfile(depth=10, reliability=BEST_EFFORT, durability=VOLATILE)
odom_sub = node.create_subscription(Odometry, odom_topic, callback, qos)
try:
    start_time = time.monotonic()
    while time.monotonic() - start_time < timeout_sec:
        executor.spin_once(timeout_sec=0.01)
        # ... check condition ...
finally:
    node.destroy_subscription(odom_sub)
    executor.remove_node(node)
```

This pattern repeats across `assert_vehicle_moved`, `assert_vehicle_stationary`,
`assert_vehicle_velocity`, `assert_vehicle_in_region`, `assert_vehicle_orientation`,
and `assert_vehicle_moved_with_ground_truth` — six times in one file alone, and
across every primitives module. The C++ side has the same problem.

**Impact**: ~60% of the primitives code is infrastructure boilerplate rather than
test logic.

### 1.2 Each Assertion Creates Its Own Executor

Each assertion function creates a *new* `SingleThreadedExecutor`, adds the node,
spins, then tears down. This means:

- The test fixture's executor is ignored
- Multiple assertions in one test create/destroy executors repeatedly
- Subscriptions from the fixture's `MessageCollector` don't receive callbacks during
  primitive assertion calls (they're on a different executor)
- There's a subtle conflict: if the node is already on the fixture's executor,
  adding it to a second executor can cause issues

This is a design tension: the primitives were designed as standalone functions,
but the fixture already manages an executor. These two approaches fight each other.

### 1.3 Type Erasure Bug in C++ `getMessages`

In `test_fixture_base.hpp:119-131`, the `getMessages<MsgT>` method is broken:

```cpp
auto collector = std::dynamic_pointer_cast<MessageCollector<MsgT>>(
    std::any_cast<std::shared_ptr<MessageCollector<MsgT>>>(it->second));
```

The `any_cast` already returns a `shared_ptr<MessageCollector<MsgT>>` — the
subsequent `dynamic_pointer_cast` to the same type is redundant. More importantly,
if the stored type doesn't match `MsgT`, the `any_cast` throws `std::bad_any_cast`
rather than returning null, so the null check on the next line is unreachable.

### 1.4 `clearMessages` is Unimplemented

`src/core/test_fixture_base.cpp:62-71` — the method logs a warning and does nothing.
This should either be implemented or removed from the API.

### 1.5 CRTP Template Parameter is Unused

`RequirementValidator<Derived>` never accesses `Derived`. The CRTP pattern is
unnecessary here — this could be a plain mixin class. The template parameter adds
noise to every test class declaration without benefit:

```cpp
// Current: verbose, Derived is never used
class MyTest : public TestFixtureBase,
               public RequirementValidator<MyTest> { };

// Simpler: same behavior
class MyTest : public TestFixtureBase,
               public RequirementValidator { };
```

### 1.6 Global Mutable Singleton (`ValidationResultCollector`)

`ValidationResultCollector::instance()` is a process-global singleton protected by
a mutex. This means:

- Parallel test processes sharing the same binary accumulate mixed results
- No way to scope results to a test suite or test run
- `clear()` is a global side effect that can race with concurrent tests
- The JSON export captures *all* results from the process, not per-suite

This is the antithesis of the test isolation design elsewhere in the codebase.

### 1.7 Enormous Public API Surface

`__init__.py` exports 100+ symbols. Users face a wall of 50+ assertion functions
that are hard to discover and distinguish. For example:

- `assert_sensor_publishing` vs `assert_publish_rate` (both check publish frequency)
- `assert_lifecycle_node_active` vs `assert_lifecycle_node_state` vs
  `assert_lifecycle_nodes_active` (singular vs plural vs specific state)

There's no guidance on which to use when, and no compositional structure to
build custom assertions from smaller pieces.

### 1.8 Inconsistent Return Types

Some assertions return `bool`, some return result structs, some return vectors
of result structs. The C++ side returns structs while equivalent Python functions
return dataclasses with different field names. There's no unified result type.

### 1.9 `nonlocal` Mutation in Callbacks

Every Python assertion function uses the pattern:

```python
latest_value = None
def callback(msg):
    nonlocal latest_value
    latest_value = msg
```

This is mutable shared state accessed from a callback thread and the main thread.
While the GIL provides some protection, it's fragile and untestable in isolation.

### 1.10 No Compositional Structure

Assertions are monolithic. You can't reuse "wait for odometry" separately from
"check distance traveled." If you need a slight variation (different message type,
different condition), you must write a new function from scratch or accept the
existing one's exact behavior.

---

## Part 2: Simplification Proposals

### 2.1 Extract a `TopicObserver` Combinator

Replace the repeated subscribe-spin-check pattern with a single reusable combinator:

```python
class TopicObserver(Generic[MsgT, R]):
    """Observe a topic, accumulate state, extract a result."""

    def __init__(self, topic: str, msg_type: Type[MsgT],
                 fold: Callable[[R, MsgT], R], initial: R):
        self.topic = topic
        self.msg_type = msg_type
        self.fold = fold
        self.initial = initial

    def run(self, node: Node, executor: SingleThreadedExecutor,
            timeout_sec: float) -> R:
        """Subscribe, spin, fold messages, return result."""
        state = self.initial
        def callback(msg):
            nonlocal state
            state = self.fold(state, msg)
        sub = node.create_subscription(self.msg_type, self.topic, callback, SENSOR_QOS)
        try:
            spin_for_duration(executor, timeout_sec)
        finally:
            node.destroy_subscription(sub)
        return state
```

This is a *fold over a message stream* — one of the most fundamental FP patterns.
All current assertion functions become thin wrappers:

```python
def observe_max_velocity(topic: str, timeout: float) -> TopicObserver[Odometry, float]:
    return TopicObserver(topic, Odometry,
        fold=lambda mx, msg: max(mx, linear_speed(msg)),
        initial=0.0)

# Usage:
max_vel = observe_max_velocity("/odom", 2.0).run(node, executor, 2.0)
assert max_vel < 0.05, f"Robot moving at {max_vel} m/s"
```

**Impact**: Eliminates ~500 lines of boilerplate across the primitives modules.

### 2.2 Use the Fixture's Executor Everywhere

Instead of primitives creating their own executors, accept an executor parameter
(or use the one from the fixture). This eliminates the
create/add/spin/remove/destroy cycle in every function.

### 2.3 Replace the Singleton with Scoped Collectors

```python
class ValidationScope:
    """Scoped validation result collection."""
    def __init__(self, name: str):
        self.name = name
        self.results: list[ValidationResult] = []

    def add(self, result: ValidationResult):
        self.results.append(result)

    def export_json(self, path: str): ...

# Used as a context manager or fixture parameter
with ValidationScope("turtlebot3_suite") as scope:
    scope.add(validate(...))
```

### 2.4 Drop the Unused CRTP Parameter

Change `RequirementValidator<Derived>` to just `RequirementValidator`. It's a
mixin that provides methods; CRTP is only needed when the base class needs to
call derived class methods, which it doesn't here.

### 2.5 Unify Result Types with a Sum Type

```python
@dataclass(frozen=True)
class TestEvidence:
    """Immutable evidence from a test observation."""
    passed: bool
    metric: str         # what was measured
    expected: str       # what was expected
    actual: str         # what was observed
    details: str = ""
    requirement_id: str = ""
```

One type instead of seven (`MovementResult`, `VelocityResult`, `SensorDataResult`,
`NavigationResult`, `LifecycleResult`, `ControllerResult`, `LocalizationResult`).

---

## Part 3: FP Testing — Hedgehog/Hypothesis Ideas

### 3.1 Core Insight: Assertions Are Properties Over Message Streams

The fundamental pattern in sim_harness is:

> "Observe a topic for N seconds, then assert a property holds over the
> collected messages."

In Hedgehog/Hypothesis terms, each message is a *sample*, and the assertion is a
*property* that should hold for all samples. This is exactly what property-based
testing frameworks are built for.

### 3.2 Composable Predicates (Inspired by Hedgehog's `Property` Type)

Instead of monolithic assertion functions, define small composable predicates:

```python
# Predicates are just functions: MsgT -> bool
def ranges_finite(scan: LaserScan) -> bool:
    return all(math.isfinite(r) or math.isinf(r) for r in scan.ranges)

def ranges_within(lo: float, hi: float) -> Callable[[LaserScan], bool]:
    return lambda scan: all(lo <= r <= hi for r in scan.ranges if math.isfinite(r))

def has_min_points(n: int) -> Callable[[LaserScan], bool]:
    return lambda scan: len(scan.ranges) >= n

# Compose them:
valid_lidar = all_of(ranges_finite, ranges_within(0.1, 100.0), has_min_points(100))

# Use against a stream:
messages = collect(node, "/scan", LaserScan, timeout=5.0)
assert_property(messages, valid_lidar, "LIDAR data is valid")
```

Compare to the current monolithic approach:

```python
# Current: one big function you can't decompose
result = assert_lidar_valid(node, "/scan", min_range=0.1, max_range=100.0,
                            min_points=100, timeout=5)
```

The composable version lets users mix and match predicates, add custom ones, and
understand exactly what's being checked.

### 3.3 Hypothesis Integration for Scenario Generation

Python's `hypothesis` library is a direct port of Hedgehog's ideas. Use it to
generate test scenarios:

```python
from hypothesis import given, settings
from hypothesis.strategies import floats, tuples

# Generate random goal poses within a region
goal_positions = tuples(
    floats(min_value=-5.0, max_value=5.0),   # x
    floats(min_value=-5.0, max_value=5.0),    # y
)

@given(goal=goal_positions)
@settings(max_examples=10)  # 10 random goals per test run
def test_navigation_reaches_any_goal(goal, sim_fixture):
    """Property: the robot can reach any reachable goal in the free space."""
    x, y = goal
    pose = make_pose_stamped(x, y)
    result = assert_reaches_goal(sim_fixture.node, "", pose, timeout=60)
    # If the goal is in free space, navigation should succeed
    if is_in_free_space(x, y, costmap):
        assert result.success, f"Failed to reach ({x}, {y}): {result.details}"
```

This gives you **shrinking for free**: when a navigation test fails, Hypothesis
automatically finds the *simplest* failing goal pose (nearest to origin, round
numbers, etc.), which makes debugging much easier — just like Hedgehog.

### 3.4 Generator-Based Sensor Validation

Instead of hardcoding sensor bounds, generate them:

```python
from hypothesis import given
from hypothesis.strategies import floats

@given(
    max_accel=floats(min_value=10.0, max_value=100.0),
    max_angular=floats(min_value=1.0, max_value=20.0),
)
def test_imu_within_physical_bounds(max_accel, max_angular, sim_fixture):
    """Property: IMU data never exceeds physical bounds, regardless of threshold."""
    messages = collect(sim_fixture.node, "/imu", Imu, timeout=3.0)
    for msg in messages:
        accel = magnitude(msg.linear_acceleration)
        assert accel < max_accel, f"Accel {accel} exceeds {max_accel}"
```

### 3.5 Stream Properties (Inspired by Hedgehog's `forAll`)

Define a `forAll` combinator over message streams:

```python
def for_all_messages(
    node: Node,
    topic: str,
    msg_type: Type[MsgT],
    property: Callable[[MsgT], bool],
    timeout_sec: float = 5.0,
    min_samples: int = 1,
    description: str = "",
) -> PropertyResult:
    """
    Assert that a property holds for all messages on a topic.

    Like Hedgehog's forAll, but over a ROS message stream instead of
    generated values. Reports the first counterexample if the property fails.
    """
    messages = collect(node, topic, msg_type, timeout=timeout_sec)
    assert len(messages) >= min_samples, (
        f"Not enough samples: got {len(messages)}, need {min_samples}"
    )
    for i, msg in enumerate(messages):
        if not property(msg):
            return PropertyResult(
                passed=False,
                counterexample=msg,
                counterexample_index=i,
                total_checked=i + 1,
                description=description,
            )
    return PropertyResult(
        passed=True,
        total_checked=len(messages),
        description=description,
    )
```

Usage becomes declarative:

```python
# Instead of:
result = assert_imu_valid(node, "/imu", max_acceleration=50.0)

# Write:
result = for_all_messages(node, "/imu", Imu,
    property=lambda msg: magnitude(msg.linear_acceleration) < 50.0,
    description="IMU acceleration within bounds")
```

When this fails, you get the **exact counterexample message** — far more useful
than "sensor data invalid."

### 3.6 Temporal Properties

Hedgehog-style properties can be extended to temporal sequences:

```python
def eventually(
    predicate: Callable[[MsgT], bool],
    within: float,
) -> StreamProperty[MsgT]:
    """The predicate becomes true for at least one message within the timeout."""
    ...

def always(
    predicate: Callable[[MsgT], bool],
) -> StreamProperty[MsgT]:
    """The predicate holds for every message in the stream."""
    ...

def monotonic(
    extract: Callable[[MsgT], float],
) -> StreamProperty[MsgT]:
    """The extracted value is monotonically increasing across the stream."""
    ...

# Usage:
assert stream("/odom", Odometry, timeout=10).satisfies(
    eventually(lambda o: distance(o.pose, goal) < 0.5, within=60)
)
assert stream("/scan", LaserScan, timeout=5).satisfies(
    always(lambda s: len(s.ranges) >= 100)
)
```

### 3.7 Practical Hypothesis Strategies for ROS

```python
import hypothesis.strategies as st

# Strategy: valid Twist command
twist_commands = st.builds(
    Twist,
    linear=st.builds(Vector3,
        x=st.floats(min_value=-2.0, max_value=2.0),
        y=st.just(0.0), z=st.just(0.0)),
    angular=st.builds(Vector3,
        x=st.just(0.0), y=st.just(0.0),
        z=st.floats(min_value=-1.0, max_value=1.0)),
)

# Strategy: valid PoseStamped within map bounds
poses_in_map = st.builds(
    make_pose_stamped,
    x=st.floats(min_value=-10, max_value=10),
    y=st.floats(min_value=-10, max_value=10),
    yaw=st.floats(min_value=-math.pi, max_value=math.pi),
)

# Strategy: QoS profiles
qos_profiles = st.builds(
    QoSProfile,
    depth=st.integers(min_value=1, max_value=100),
    reliability=st.sampled_from([RELIABLE, BEST_EFFORT]),
)
```

### 3.8 Shrinking Integration Test Failures

The most valuable Hedgehog feature is *automatic shrinking*. For sim testing:

```python
@given(
    velocity=st.floats(min_value=0.1, max_value=2.0),
    duration=st.floats(min_value=1.0, max_value=10.0),
)
def test_distance_proportional_to_velocity(velocity, duration, sim_fixture):
    """Property: distance ≈ velocity × time (within tolerance)."""
    result = assert_vehicle_moved(
        sim_fixture.node, "robot", min_distance=0.0,
        velocity=velocity, timeout_sec=duration
    )
    expected = velocity * duration
    # Allow 30% tolerance for physics sim
    assert result.distance_moved > expected * 0.7, (
        f"At v={velocity}, t={duration}: expected ~{expected}m, got {result.distance_moved}m"
    )
```

When this fails, Hypothesis automatically shrinks to the **minimal failing case**
(e.g., `velocity=0.1, duration=1.0`), making it immediately clear whether
the issue is low-velocity deadband, slow start, or a physics bug.

---

## Part 4: Concrete Refactoring Roadmap

### Phase 1: Extract Infrastructure (Low Risk)

1. Create `TopicObserver` combinator to replace per-function boilerplate
2. Remove unused CRTP parameter from `RequirementValidator`
3. Fix or remove `clearMessages` stub
4. Fix `getMessages` type erasure in C++ `TestFixtureBase`

### Phase 2: Composable Predicates (Medium Risk)

1. Define predicate type: `Callable[[MsgT], bool]`
2. Add combinators: `all_of`, `any_of`, `for_all_messages`
3. Rewrite existing assertions as predicate compositions
4. Keep old functions as thin wrappers for backward compatibility

### Phase 3: Property-Based Testing (New Capability)

1. Add `hypothesis` as a test dependency
2. Define ROS message strategies (`twist_commands`, `poses_in_map`, etc.)
3. Write property tests for sensor validation
4. Write property tests for navigation scenarios
5. Add `for_all_messages` / `eventually` / `always` stream combinators

### Phase 4: Scoped Validation (Breaking Change)

1. Replace singleton `ValidationResultCollector` with `ValidationScope`
2. Pass scope explicitly or via fixture
3. Each test suite gets its own scope and JSON export

---

## Part 5: Summary of Recommendations

| Priority | Issue | Recommendation |
|----------|-------|----------------|
| High | Boilerplate in primitives | Extract `TopicObserver` fold combinator |
| High | Each assertion creates own executor | Accept executor as parameter |
| High | `getMessages` type erasure bug | Fix `any_cast` / remove `dynamic_pointer_cast` |
| Medium | Global singleton collector | Replace with scoped `ValidationScope` |
| Medium | Unused CRTP parameter | Drop to plain mixin |
| Medium | Monolithic assertions | Decompose into composable predicates |
| Medium | 100+ API surface | Organize into submodules, export less |
| Low | `clearMessages` unimplemented | Implement or remove |
| Low | `nonlocal` mutation pattern | Use `TopicObserver` fold instead |
| New | Property-based testing | Add Hypothesis + stream properties |
| New | Generator-driven scenarios | Use Hypothesis strategies for goals/velocities |
| New | Automatic shrinking | Leverage Hypothesis for minimal failing cases |
| New | Temporal stream combinators | `always`, `eventually`, `monotonic` |
