Core API
========

Core classes and functions for sim_harness.

SimTestFixture
--------------

Base test fixture for ROS 2 simulation tests.

**Properties:** ``node`` - The ROS 2 node instance

**Methods:**

- ``get_logger()`` - Get the node's logger
- ``spin_for_duration(seconds)`` - Spin the node
- ``create_message_collector(topic, msg_type, ...)`` - Create collector
- ``assert_requirement(...)`` - Record and assert requirement
- ``validate_requirement(...)`` - Record without asserting

SimulationTestFixture
---------------------

Test fixture with automatic simulation lifecycle.

**Class Attributes:**

- ``LAUNCH_PACKAGE`` - ROS 2 package with launch file
- ``LAUNCH_FILE`` - Launch file name
- ``LAUNCH_ARGS`` - Launch arguments dict
- ``STARTUP_TIMEOUT`` - Max startup wait (default: 60.0)

MessageCollector
----------------

Collects messages from a ROS 2 topic.

**Methods:**

- ``get_messages()`` - Get all messages
- ``get_latest()`` - Get most recent message
- ``message_count()`` - Get count
- ``clear()`` - Clear buffer
- ``wait_for_messages(count, timeout)`` - Wait for messages

ValidationResultCollector
-------------------------

Singleton collector for validation results.

**Methods:**

- ``instance()`` - Get singleton
- ``add_result(result)`` - Add result
- ``get_results()`` - Get all results
- ``print_summary()`` - Print summary
- ``export_to_json(path)`` - Export to JSON

ValidationScope
---------------

Scoped validation result collection.  Replaces the global singleton for
parallel-safe test suites.

**Constructor:** ``ValidationScope(name, parent=None)``

**Methods:**

- ``add_result(result)`` - Add result (propagates to parent if set)
- ``clear()`` - Clear results
- ``get_results()`` - Get all results
- ``get_counts()`` - Get (passed, failed) counts
- ``export_to_json(path)`` - Export to JSON
- ``print_summary()`` - Print summary

TopicObserver
-------------

A fold combinator over ROS 2 message streams.  The fundamental building block
for stream-based assertions.

**Constructor:** ``TopicObserver(topic, msg_type, initial, step, extract, qos=None)``

- ``initial`` - Seed value for the fold
- ``step`` - ``(accumulator, message) -> accumulator``
- ``extract`` - ``accumulator -> result``

**Methods:**

- ``run(node, executor, timeout_sec)`` - Execute the fold, return ``ObservationResult``
- ``run_standalone(node, timeout_sec)`` - Run with a temporary executor
- ``map(f)`` - Transform the result with a pure function

**Factory functions:**

- ``collect_messages(topic, msg_type)`` - Collect all messages into a list
- ``count_messages(topic, msg_type)`` - Count messages
- ``latest_message(topic, msg_type)`` - Capture the latest message
- ``track_max(topic, msg_type, extract)`` - Track the maximum of an extracted value
- ``track_timestamps(topic, msg_type)`` - Record wall-clock receipt times

**ParallelObserver:** Run multiple observers in a single spin period.

.. code-block:: python

   odom_result, scan_result = ParallelObserver(
       track_max("/odom", Odometry, linear_speed),
       count_messages("/scan", LaserScan),
   ).run(node, executor, 5.0)

Stream Properties
-----------------

Hedgehog-inspired property checking over message streams.

**Functions:**

- ``for_all_messages(node, executor, topic, msg_type, predicate, ...)`` - Property
  must hold for every message.  Returns ``PropertyResult`` with counterexample on failure.
- ``eventually(node, executor, topic, msg_type, predicate, ...)`` - Property must
  hold for at least one message.
- ``monotonic(node, executor, topic, msg_type, extract_value, ...)`` - Extracted
  values must be monotonically non-decreasing.

**Combinators:**

- ``all_of(*predicates)`` - All predicates must hold
- ``any_of(*predicates)`` - At least one must hold
- ``negate(predicate)`` - Invert a predicate

Composable Predicates
---------------------

Small, reusable predicate functions (``Msg -> bool``) for common sensor and
vehicle checks.  Compose them with ``all_of``, ``any_of``, ``negate``.

**LaserScan:** ``scan_has_min_points(n)``, ``scan_ranges_within(lo, hi)``,
``scan_nan_ratio_below(threshold)``, ``scan_has_range_count(n)``

**IMU:** ``imu_accel_within(max)``, ``imu_gyro_within(max)``, ``imu_no_nan()``

**Image:** ``image_has_data()``, ``image_dimensions(w, h)``, ``image_encoding(enc)``

**GPS:** ``gps_no_nan()``, ``gps_in_bounds(min_lat, max_lat, min_lon, max_lon)``,
``gps_has_fix()``

**JointState:** ``joints_present(names)``, ``joints_no_nan()``

**Odometry:** ``odom_position_finite()``, ``odom_velocity_below(max)``,
``odom_in_region(min_x, max_x, min_y, max_y)``

Property-Based Testing (Hypothesis)
------------------------------------

Three-tier Hypothesis integration for simulation testing.
Requires ``pip install hypothesis`` (or ``pip install sim-harness[hypothesis]``).

**sim_property(max_examples, deadline, nightly, ...)**

Pre-configured ``@settings`` decorator for sim tests.  No deadline, persistent
database, ``too_slow`` suppressed.  Set ``SIM_HARNESS_NIGHTLY=1`` for 10x more
examples.

**Tier 1 — Recorded Data Helpers:**

- ``check_recorded_property(data, fn, description)`` - Check predicate over every item
- ``check_recorded_eventually(data, fn, description)`` - At least one must satisfy
- ``check_recorded_monotonic(data, extract, strict, description)`` - Monotonicity check
- ``hypothesis_check_recorded(data, strategy, fn, ...)`` - Hypothesis generates parameters,
  checks against recorded data with shrinking

All raise ``PropertyFailure`` (subclass of ``AssertionError``) with
``.counterexample`` and ``.failure_index`` attributes.

**ROS Message Strategies** (``sim_harness.core.strategies``):

Geometry: ``point_strategy``, ``vector3_strategy``, ``quaternion_strategy``,
``yaw_quaternion_strategy``, ``pose_strategy``

Velocity: ``twist_strategy`` (ground robot), ``twist_strategy_3d`` (6-DOF)

Navigation: ``navigation_goal_2d``, ``waypoints_strategy``

Parameters: ``duration_strategy``, ``rate_strategy``, ``threshold_strategy``,
``angle_strategy``, ``speed_strategy``, ``distance_strategy``
