Assertion Primitives
====================

Pre-built assertion functions for validating ROS 2 simulation behavior.
These primitives provide a DSL-style interface for common testing patterns.

All assertions follow a consistent pattern:

- Accept a ``node`` parameter (your test's ROS 2 node)
- Return structured result objects with ``success``/``valid`` status and ``details``
- Handle timeouts gracefully with informative error messages

Sensor Assertions
-----------------

Functions for validating sensor data from ROS 2 topics.

.. automodule:: sim_harness.primitives.sensor_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Vehicle Assertions
------------------

Functions for validating robot movement and state using odometry.

.. automodule:: sim_harness.primitives.vehicle_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Navigation Assertions
---------------------

Functions for validating robot navigation behavior with Nav2.

.. automodule:: sim_harness.primitives.navigation_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Lifecycle Assertions
--------------------

Functions for validating ROS 2 lifecycle nodes and ros2_control controllers.

.. automodule:: sim_harness.primitives.lifecycle_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Service Assertions
------------------

Functions for validating ROS 2 services, actions, and node availability.

.. automodule:: sim_harness.primitives.service_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Timing Assertions
-----------------

Functions for validating publish rates, message latency, and TF availability.

.. automodule:: sim_harness.primitives.timing_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Perception Assertions
---------------------

Functions for validating object detection and perception systems.

.. automodule:: sim_harness.primitives.perception_assertions
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Usage Examples
--------------

Basic sensor validation:

.. code-block:: python

   from sim_harness.primitives import assert_lidar_valid, assert_imu_valid

   def test_sensors(self):
       # Validate LIDAR data
       result = assert_lidar_valid(
           self.node,
           topic="/scan",
           min_range=0.1,
           max_range=30.0,
           min_points=100,
           timeout_sec=5.0
       )
       assert result.valid, result.details

       # Validate IMU data
       result = assert_imu_valid(
           self.node,
           topic="/imu/data",
           max_acceleration=20.0,
           timeout_sec=5.0
       )
       assert result.valid, result.details

Vehicle movement testing:

.. code-block:: python

   from sim_harness.primitives import assert_vehicle_moved, assert_vehicle_stationary

   def test_vehicle_moves(self):
       result = assert_vehicle_moved(
           self.node,
           vehicle_id="robot_01",
           min_distance=1.0,
           velocity=0.5,
           timeout_sec=10.0
       )
       assert result.success, result.details

   def test_vehicle_stops(self):
       stopped = assert_vehicle_stationary(
           self.node,
           vehicle_id="robot_01",
           velocity_threshold=0.01,
           duration_sec=2.0
       )
       assert stopped

Lifecycle and controller validation:

.. code-block:: python

   from sim_harness.primitives import (
       assert_lifecycle_node_active,
       assert_nav2_active,
       assert_controller_active
   )

   def test_nav2_stack(self):
       # Check all Nav2 nodes are active
       results = assert_nav2_active(self.node, timeout_sec=60.0)
       for result in results:
           assert result.success, result.details

   def test_controller(self):
       result = assert_controller_active(
           self.node,
           controller_manager_name="controller_manager",
           controller_name="joint_trajectory_controller",
           timeout_sec=30.0
       )
       assert result.success, result.details

Navigation testing:

.. code-block:: python

   from geometry_msgs.msg import PoseStamped
   from sim_harness.primitives import assert_navigation_action_succeeds

   def test_navigation(self):
       goal = PoseStamped()
       goal.header.frame_id = "map"
       goal.pose.position.x = 5.0
       goal.pose.position.y = 3.0

       result = assert_navigation_action_succeeds(
           self.node,
           goal_pose=goal,
           timeout_sec=120.0
       )
       assert result.success, result.details
