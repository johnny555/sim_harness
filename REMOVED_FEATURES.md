# Removed Features

Features removed to reduce complexity. This document captures the high-level
ideas so they can be re-implemented if needed.

## 1. Stream Properties (`core/stream_properties.py` — 289 lines)

Hedgehog-inspired property combinators that check invariants over live ROS
topic streams.

- **`for_all_messages(node, executor, topic, msg_type, predicate, ...)`** —
  Subscribe to a topic, collect messages for a window, assert a predicate
  holds on *every* message. Returns a `PropertyResult` with the first
  counterexample (message index + content) when the property is violated.
- **`eventually(..., predicate, ...)`** — Same collection pattern, but passes
  if the predicate holds for *at least one* message. Useful for convergence
  tests ("the robot eventually reaches the goal").
- **`monotonic(..., extract_value, ...)`** — Asserts that an extracted numeric
  value is non-decreasing (or strictly increasing) across messages. Useful
  for timestamp ordering, cumulative distance, sequence numbers.
- **`PropertyResult`** dataclass — carried `passed`, `total_checked`,
  `first_failure_index`, `counterexample`, and a human-readable
  `counterexample_details` string.

**Why removed:** Only the predicate combinators (`all_of`/`any_of`/`negate`)
were used by the actual primitives. Those 15 lines moved into `predicates.py`.
The stream-level functions were unused.

## 2. Hypothesis Integration (`core/sim_property.py` — 345 lines)

A three-tier framework for property-based testing of simulations using the
Hypothesis library.

- **`sim_property(max_examples=...)`** — Decorator wrapping Hypothesis
  `@settings` with sim-friendly defaults: no deadline, persistent example
  database, suppressed slow-health-check warnings, nightly mode via
  `SIM_HARNESS_NIGHTLY` env var.
- **Tier 1 helpers** (check over recorded data, no sim re-run):
  - `check_recorded_property(data, property_fn)` — assert property over a
    list of already-collected messages.
  - `check_recorded_eventually(data, property_fn)` — at least one message
    satisfies the predicate.
  - `check_recorded_monotonic(data, extract, strict=False)` — extracted
    values are non-decreasing.
  - `hypothesis_check_recorded(data, strategy, property_fn)` — Hypothesis
    generates random parameters, checks them against recorded data.

**Why removed:** No primitives or existing tests used this. Hypothesis is an
optional dependency that adds integration complexity. The three-tier concept
is sound if property-based testing is later needed.

## 3. Hypothesis Strategies (`core/strategies.py` — 328 lines)

Type-safe Hypothesis strategies for generating physically plausible ROS 2
messages.

- **Geometry:** `point_strategy`, `vector3_strategy`,
  `quaternion_strategy`, `yaw_quaternion_strategy`, `pose_strategy`,
  `pose_stamped_strategy`
- **Commands:** `twist_strategy(max_linear, max_angular)`,
  `navigation_goal_2d(x_bounds, y_bounds)`
- **Parameters:** `speed_strategy`, `threshold_strategy`,
  `distance_strategy`, `angle_strategy`, `duration_strategy`
- **Composite:** `waypoint_list_strategy` (list of poses for path planning)

All strategies generate values within configurable physical bounds (e.g.
speeds 0–2 m/s, distances 0–100 m) so Hypothesis doesn't waste time on
impossible inputs.

**Why removed:** Only used by the property-based testing example. No
production tests depended on these strategies.

## 4. Checks Re-export Layer (`checks/` package — 245 lines)

A user-facing package that organized assertions and predicates into five
purpose-based modules:

- `checks.readiness` — service, lifecycle, controller, nav2 availability
- `checks.sensors` — sensor validation + publish rate/latency + sensor predicates
- `checks.motion` — vehicle movement/velocity + odometry predicates
- `checks.navigation` — goal-reaching, path-following, costmaps
- `checks.perception` — object detection, region clearing

Each module was a thin re-export from `primitives/` and `core/predicates.py`.
The `__init__.py` also re-exported `all_of`/`any_of`/`negate`.

**Why removed:** 245 lines of pure indirection with zero logic. The same
organizational benefit is achieved with comments in the top-level
`__init__.py`, without an extra package that users have to understand sits
between them and the actual code.

## 5. Hypothesis Re-export (`core/hypothesis.py` — 61 lines)

Single-file re-export of `sim_property` + `strategies` for a convenient
`from sim_harness.core.hypothesis import ...` import path.

**Why removed:** Both source modules were deleted.

## 6. Property-Based Testing Example (`examples/test_property_based.py` — 557 lines)

Demonstrated all three Hypothesis tiers:

- **Tier 1:** Collect messages once, check many properties against recorded
  data (cheap). Example: "every LIDAR scan has ≥10 finite ranges."
- **Tier 2:** Hypothesis generates entire scenarios (goals, poses). Each
  example re-runs sim. Example: "for any goal within bounds, the robot moves
  toward it."
- **Tier 3:** Sim stays running, Hypothesis varies commands within a session.
  Example: "for any Twist command, odometry stays finite."

**Why removed:** Depended entirely on the deleted Hypothesis infrastructure.

## 7. Readiness Check Framework (`core/readiness_check.py` — 669 lines)

A configurable checklist builder for verifying robot readiness before tests.

- **`ReadinessCheck(node)`** — Builder that chains `.check_topics()`,
  `.check_nodes()`, `.check_services()`, `.check_transforms()`,
  `.check_lifecycle_nodes()`, `.check_controllers()`, `.check_custom()`,
  then `.run()` to execute all checks with timeouts and produce a
  `CheckResult`.
- **`CheckResult`** — Aggregated result with pass/fail/skip counts,
  `all_passed` flag, `failed_checks` list, and `by_category()` filter.
- **`CheckCategory` enum** — TOPIC, NODE, SERVICE, TRANSFORM, LIFECYCLE,
  CONTROLLER, CUSTOM.
- **`create_standard_check()`** — Factory for common robot configs
  (topics, nodes, services, transforms in one call).

**Why removed:** Exported in `__init__.py` and documented, but never actually
used by any test, example, or other module. Users preferred calling individual
assertion functions (`assert_service_available`, `assert_lifecycle_node_active`,
etc.) directly. The builder pattern added a layer of abstraction over
functionality already available as simple function calls.

## 8. Core Module Layer (`core/` package — 1,551 lines, 8 files)

Intermediate abstraction layer between user-facing API and assertion primitives:

- **`TopicObserver`** — Generic subscribe/spin/check pattern with `run_standalone()`
  and fold-based reducers (`collect_messages`, `count_messages`, `latest_message`,
  `track_max`, `track_timestamps`). `ParallelObserver` ran multiple observers.
- **`predicates.py`** — Composable predicate combinators (`all_of`, `any_of`,
  `negate`) for building complex sensor checks.
- **`test_isolation.py`** — `TestIsolationConfig` / `apply_test_isolation()` /
  `generate_test_node_name()` for unique ROS domain IDs per test.
- **`message_collector.py`** — Thread-safe message collector with callbacks,
  filtering, and rate statistics.
- **`spin_helpers.py`** — `spin_for_duration`, `spin_until_condition`,
  `spin_until_messages_received`.
- **`test_fixture.py`** — SimTestFixture inheriting from RequirementValidator.
- **`simulation_fixture.py`** — SimulationTestFixture alias.

**Why removed:** The 6-file flat structure (`fixture.py`, `collector.py`,
`spin.py`, `assertions.py`, `nav2.py`, `perception.py`) inlines all needed
functionality. TopicObserver's subscribe/spin/check pattern is just a few lines
when inlined. Test isolation is a single `os.environ` call. Predicates were
only used internally. The simpler MessageCollector drops thread-safety
overhead that wasn't needed (ROS callbacks are single-threaded per executor).

## 9. Primitives Module Layer (`primitives/` package — 3,047 lines, 8 files)

Individual assertion modules, each with full TopicObserver integration:

- **`sensor_assertions.py`** (433 lines) — LIDAR, GPS, IMU, camera, joint state
- **`timing_assertions.py`** (308 lines) — publish rate, latency, TF availability
- **`service_assertions.py`** (293 lines) — service/action/node/parameter checks
- **`vehicle_assertions.py`** (631 lines) — movement, velocity, region, orientation
- **`lifecycle_assertions.py`** (515 lines) — lifecycle state, controllers, Nav2
- **`navigation_assertions.py`** (411 lines) — goal reaching, path following, costmaps
- **`perception_assertions.py`** (455 lines) — object detection, region clearing

**Why removed:** All assertion logic merged into `assertions.py` (service,
sensor, timing, vehicle), `nav2.py` (lifecycle, navigation), and
`perception.py` (detection). Verbose docstrings trimmed, TopicObserver
dependency eliminated by inlining 3-line subscribe/spin/collect patterns.
Total: 3,047 lines → 1,420 lines across three files.
