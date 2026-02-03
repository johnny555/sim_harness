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
   result = assert_vehicle_moved(node, min_distance=0.5)
   assert result.success, f"Robot didn't move: {result.details}"

   # Verify robot is stationary
   is_stopped = assert_vehicle_stationary(node, duration_sec=2.0)
   assert is_stopped

Ground Truth Validation
-----------------------

For precise validation, you can compare odometry against Gazebo's ground truth.
This helps detect odometry drift and confirms the robot actually moved in simulation.

.. code-block:: python

   from sim_harness import assert_vehicle_moved_with_ground_truth

   result = assert_vehicle_moved_with_ground_truth(
       node,
       gazebo_model_name="turtlebot3_waffle",
       min_distance=1.0,
       world_name="turtlebot3_world"
   )

   assert result.success, f"Robot didn't move: {result.details}"

The ``MovementResult`` from ground truth validation includes:

- ``success`` - Whether the robot moved the required distance (per ground truth)
- ``distance_moved`` - Distance according to odometry
- ``ground_truth_distance`` - Distance according to Gazebo
- ``odom_error`` - Position error between odom and ground truth

See :doc:`simulation_control` for more on direct Gazebo ground truth access.

Navigation Assertions
---------------------

Lifecycle, controller, and navigation assertions live in ``sim_harness.nav2``:

.. code-block:: python

   from sim_harness.nav2 import (
       assert_lifecycle_node_active,
       assert_controller_active,
       assert_nav2_active,
       assert_reaches_goal,
   )

   # Check lifecycle node is active
   result = assert_lifecycle_node_active(node, "controller_server")
   assert result.success

   # Check Nav2 stack is ready
   results = assert_nav2_active(node, timeout_sec=30.0)
   assert all(r.success for r in results)

   # Verify robot reaches a goal
   result = assert_reaches_goal(
       node, goal_pose=goal, tolerance=0.3, timeout_sec=60.0
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

Perception Assertions
---------------------

Object detection and safety zone checks live in ``sim_harness.perception``:

.. code-block:: python

   from geometry_msgs.msg import Point
   from sim_harness.perception import assert_object_detected, assert_region_clear

   # Check an object is detected near a position
   pos = Point(x=2.0, y=1.0, z=0.0)
   result = assert_object_detected(node, "/detections", pos, search_radius=1.0)
   assert result.detected, result.details

   # Verify a safety zone is clear
   center = Point(x=0.0, y=0.0, z=0.0)
   assert assert_region_clear(node, "/detections", center, radius=0.5)

Assertion Results
-----------------

Most assertions return result objects with:

- ``success`` - Boolean indicating pass/fail
- ``details`` - Human-readable description
- Additional context-specific fields (distances, times, etc.)

**Example result handling:**

.. code-block:: python

   result = assert_vehicle_moved(node, min_distance=1.0)

   if not result.success:
       print(f"FAIL: {result.details}")
       print(f"Distance: {result.distance_moved}m")
   else:
       print(f"PASS: Robot moved {result.distance_moved:.2f}m")
