Assertions
==========

Pre-built assertion functions for common robotics testing scenarios.

Sensor Assertions
-----------------

Validate that sensors are publishing valid data:

.. code-block:: python

   from sim_harness import (
       assert_sensor_publishing,
       assert_lidar_valid,
       assert_imu_valid,
       assert_gps_valid,
   )
   from sensor_msgs.msg import LaserScan

   # Check a topic is publishing
   result = assert_sensor_publishing(node, "/scan", LaserScan, timeout=5.0)
   assert result.success, f"LIDAR not publishing: {result.details}"

   # Validate LIDAR data quality
   result = assert_lidar_valid(node, "/scan", min_ranges=100, max_range=10.0)
   assert result.success

Vehicle Assertions
------------------

Validate robot movement and state:

.. code-block:: python

   from sim_harness import (
       assert_vehicle_moved,
       assert_vehicle_stationary,
       assert_vehicle_velocity,
       assert_vehicle_in_region,
   )

   # Check robot moved at least 0.5 meters
   result = assert_vehicle_moved(node, "robot_01", min_distance=0.5)
   assert result.success, f"Robot didn't move: {result.details}"
   print(f"Moved {result.distance_moved:.2f}m")

   # Verify robot is stationary
   is_stopped = assert_vehicle_stationary(node, "robot_01", duration_sec=2.0)
   assert is_stopped

   # Check robot reached target velocity
   result = assert_vehicle_velocity(node, "robot_01", target_velocity=1.0)
   assert result.success

Ground Truth Validation
-----------------------

For precise validation, you can compare odometry against Gazebo's ground truth.
This helps detect odometry drift and confirms the robot actually moved in simulation.

.. code-block:: python

   from sim_harness import assert_vehicle_moved_with_ground_truth

   result = assert_vehicle_moved_with_ground_truth(
       node,
       vehicle_id="turtlebot3",              # ROS namespace
       gazebo_model_name="turtlebot3_waffle", # Model name in Gazebo
       min_distance=1.0,
       world_name="turtlebot3_world"
   )

   # Check robot actually moved (ground truth)
   assert result.success, f"Robot didn't move: {result.details}"

   # Validate odometry accuracy
   assert result.odom_error < 0.1, f"Odom drift too high: {result.odom_error}m"

   # Access detailed results
   print(f"Ground truth distance: {result.ground_truth_distance:.2f}m")
   print(f"Odometry distance: {result.distance_moved:.2f}m")
   print(f"Odom error: {result.odom_error:.3f}m")

The ``MovementResult`` from ground truth validation includes:

- ``success`` - Whether the robot moved the required distance (per ground truth)
- ``distance_moved`` - Distance according to odometry
- ``ground_truth_distance`` - Distance according to Gazebo
- ``odom_error`` - Position error between odom and ground truth
- ``ground_truth_start`` / ``ground_truth_end`` - Exact positions from Gazebo

See :doc:`simulation_control` for more on direct Gazebo ground truth access.

Navigation Assertions
---------------------

Validate navigation stack components:

.. code-block:: python

   from sim_harness import (
       assert_lifecycle_node_active,
       assert_controller_active,
       assert_nav2_active,
       assert_reaches_goal,
   )

   # Check lifecycle node is active
   result = assert_lifecycle_node_active(node, "controller_server")
   assert result.success

   # Check Nav2 stack is ready
   result = assert_nav2_active(node, timeout_sec=30.0)
   assert result.success

   # Verify robot reaches a goal
   result = assert_reaches_goal(
       node, goal_x=2.0, goal_y=1.0, tolerance=0.3, timeout_sec=60.0
   )
   assert result.success

Service Assertions
------------------

Validate services and action servers:

.. code-block:: python

   from sim_harness import (
       assert_service_available,
       assert_action_server_available,
       assert_node_running,
   )

   # Check service is available
   result = assert_service_available(node, "/get_map", timeout_sec=5.0)
   assert result.success

   # Check action server is ready
   result = assert_action_server_available(
       node, "/navigate_to_pose", timeout_sec=10.0
   )
   assert result.success

Timing Assertions
-----------------

Validate timing and latency requirements:

.. code-block:: python

   from sim_harness import assert_publish_rate, assert_transform_available

   # Verify topic publishes at expected rate
   result = assert_publish_rate(
       node, "/scan", expected_hz=10.0, tolerance_hz=2.0
   )
   assert result.success

   # Check TF transform is available
   result = assert_transform_available(
       node, "map", "base_link", timeout_sec=5.0
   )
   assert result.success

FP-Inspired Alternative: Composable Predicates
-----------------------------------------------

For more flexible validation, you can compose small predicates instead of
using monolithic assertion functions:

.. code-block:: python

   from sim_harness.core.stream_properties import for_all_messages, all_of
   from sim_harness.core.predicates import (
       scan_has_min_points, scan_ranges_within, scan_nan_ratio_below,
       imu_no_nan, imu_accel_within,
       odom_position_finite, odom_velocity_below,
   )

   # Compose a LIDAR quality check from small pieces
   valid_lidar = all_of(
       scan_has_min_points(100),
       scan_ranges_within(0.1, 30.0),
       scan_nan_ratio_below(0.05),
   )

   result = for_all_messages(
       self.node, self.executor, "/scan", LaserScan,
       predicate=valid_lidar,
       timeout_sec=5.0,
   )
   assert result.passed, result.counterexample_details

When a property fails, ``result.counterexample`` contains the exact message
that violated it, and ``result.first_failure_index`` tells you where in the
stream it occurred.

See :doc:`writing_tests` for the full property-based testing guide.

Assertion Results
-----------------

Most assertions return result objects with:

- ``success`` - Boolean indicating pass/fail
- ``details`` - Human-readable description
- Additional context-specific fields (distances, times, etc.)

**Example result handling:**

.. code-block:: python

   result = assert_vehicle_moved(node, "robot", min_distance=1.0)

   if not result.success:
       print(f"FAIL: {result.details}")
       print(f"Start: {result.start_position}")
       print(f"End: {result.end_position}")
       print(f"Distance: {result.distance_moved}m")
   else:
       print(f"PASS: Robot moved {result.distance_moved:.2f}m")
