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
