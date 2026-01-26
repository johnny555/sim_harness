Writing Tests
=============

This guide covers the patterns and best practices for writing sim_harness tests.

Test Structure
--------------

A typical test file follows this structure:

.. code-block:: python

   #!/usr/bin/env python3
   """Description of what this test file covers."""

   import pytest
   from sensor_msgs.msg import LaserScan

   from sim_harness import SimTestFixture


   class TestSensorIntegration(SimTestFixture):
       """Test class docstring - describes the test suite."""

       SCAN_TOPIC = "/scan"

       def test_something(self):
           """Test method docstring - describes this specific test."""
           collector = self.create_message_collector(self.SCAN_TOPIC, LaserScan)
           self.spin_for_duration(3.0)
           messages = collector.get_messages()
           assert len(messages) > 0

Test Categories
---------------

**Basic Tests** - Verify fundamental functionality:

.. code-block:: python

   def test_basic_topic_publishes(self):
       collector = self.create_message_collector("/scan", LaserScan)
       self.spin_for_duration(3.0)
       assert len(collector.get_messages()) > 0

**Integration Tests** - Verify multiple components:

.. code-block:: python

   @pytest.mark.integration
   def test_navigation_stack(self):
       nodes = ["controller_server", "planner_server"]
       for node in nodes:
           result = assert_lifecycle_node_active(self.node, node)
           assert result.success

**Motion Tests** - Involve robot movement:

.. code-block:: python

   def test_robot_moves_forward(self):
       odom = self.create_message_collector("/odom", Odometry)
       self.spin_for_duration(1.0)
       initial_x = odom.get_messages()[-1].pose.pose.position.x
       # Send velocity commands...
       # Check distance moved

Using Markers
-------------

.. code-block:: bash

   pytest -m integration      # Run only integration tests
   pytest -m "not slow"       # Skip slow tests
   pytest -k "basic"          # Run basic tests only

Best Practices
--------------

.. tip::

   Keep tests focused - each test should verify one thing.

.. tip::

   Use descriptive names - ``test_lidar_publishes_at_10hz`` is better than ``test_lidar``.

.. warning::

   Avoid test interdependence - tests should not depend on other tests running first.
