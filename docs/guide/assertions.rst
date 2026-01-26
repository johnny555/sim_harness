Assertions
==========

Pre-built assertion functions for common robotics testing scenarios.

Sensor Assertions
-----------------

.. code-block:: python

   from sim_harness import assert_topic_published, assert_sensor_data_valid

   result = assert_topic_published(node, "/scan", LaserScan, timeout_sec=5.0)
   assert result.success

Vehicle Assertions
------------------

.. code-block:: python

   from sim_harness import assert_vehicle_moved, assert_vehicle_stationary

   result = assert_vehicle_moved(node, "robot_01", min_distance=0.5)
   assert result.success

Navigation Assertions
---------------------

.. code-block:: python

   from sim_harness import assert_lifecycle_node_active

   result = assert_lifecycle_node_active(node, "controller_server")
   assert result.success

Assertion Results
-----------------

Most assertions return result objects with:

- ``success`` - Boolean
- ``details`` - Human-readable description
- Additional context-specific fields
