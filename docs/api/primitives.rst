Check Library
=============

Pre-built check functions for validating ROS 2 simulation behavior.
All checks follow a consistent pattern:

- Accept a ``node`` parameter (your test's ROS 2 node)
- Return structured result objects with an ``.ok`` property and ``details``
- Handle timeouts gracefully with informative messages

checks.py — Sensors, Timing, Services, Motion
----------------------------------------------

.. automodule:: sim_harness.checks
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

nav2.py — Lifecycle, Controllers, Navigation
---------------------------------------------

.. automodule:: sim_harness.nav2
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

perception.py — Object Detection, Region Checks
------------------------------------------------

.. automodule:: sim_harness.perception
   :members:
   :undoc-members:
   :show-inheritance:
   :member-order: bysource

Usage Examples
--------------

Basic sensor validation:

.. code-block:: python

   from sim_harness.checks import check_lidar_valid, check_imu_valid

   def test_sensors(self):
       result = check_lidar_valid(
           self.node, topic="/scan",
           min_range=0.1, max_range=30.0,
           min_points=100, timeout_sec=5.0
       )
       assert result.ok, result.details

       result = check_imu_valid(
           self.node, topic="/imu/data",
           max_acceleration=20.0, timeout_sec=5.0
       )
       assert result.ok, result.details

Vehicle movement testing:

.. code-block:: python

   from sim_harness.checks import check_vehicle_moved, check_vehicle_stationary

   def test_vehicle_moves(self):
       result = check_vehicle_moved(
           self.node, min_distance=1.0, timeout_sec=10.0
       )
       assert result.ok, result.details

Lifecycle and controller validation:

.. code-block:: python

   from sim_harness.nav2 import (
       check_lifecycle_node_active,
       check_nav2_active,
       check_controller_active,
   )

   def test_nav2_stack(self):
       results = check_nav2_active(self.node, timeout_sec=60.0)
       for result in results:
           assert result.ok, result.details

   def test_controller(self):
       result = check_controller_active(
           self.node,
           controller_manager_name="controller_manager",
           controller_name="joint_trajectory_controller",
       )
       assert result.ok, result.details

Navigation testing:

.. code-block:: python

   from geometry_msgs.msg import PoseStamped
   from sim_harness.nav2 import check_navigation_action_succeeds

   def test_navigation(self):
       goal = PoseStamped()
       goal.header.frame_id = "map"
       goal.pose.position.x = 5.0
       goal.pose.position.y = 3.0

       result = check_navigation_action_succeeds(
           self.node, goal_pose=goal, timeout_sec=120.0
       )
       assert result.ok, result.details

Perception testing:

.. code-block:: python

   from geometry_msgs.msg import Point
   from sim_harness.perception import check_object_detected, check_region_clear

   def test_detects_obstacle(self):
       pos = Point(x=2.0, y=1.0, z=0.0)
       result = check_object_detected(
           self.node, "/detections", pos, search_radius=1.0
       )
       assert result.ok, result.details

   def test_safety_zone(self):
       center = Point(x=0.0, y=0.0, z=0.0)
       assert check_region_clear(
           self.node, "/detections", center, radius=0.5
       ), "Safety zone violated"
