Your First Test
===============

Let's write a simple test for a ROS 2 robot.

Create a Test File
------------------

Create a new file ``test_my_robot.py``:

.. code-block:: python

   #!/usr/bin/env python3
   """My first sim_harness test."""

   import pytest
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry

   from sim_harness import SimTestFixture


   class TestMyRobot(SimTestFixture):
       """Test suite for my robot."""

       def test_lidar_publishes(self):
           """Verify LIDAR sensor is publishing data."""
           # Create a message collector for the scan topic
           collector = self.create_message_collector(
               "/scan",
               LaserScan,
               key="scan"
           )

           # Wait for messages (spins the ROS node)
           self.spin_for_duration(5.0)

           # Get collected messages
           messages = collector.get_messages()

           # Assert we received data
           assert len(messages) > 0, "No LIDAR messages received"

           # Check data quality
           scan = messages[-1]
           assert len(scan.ranges) > 0, "LIDAR scan has no ranges"

       def test_odometry_publishes(self):
           """Verify odometry is being published."""
           collector = self.create_message_collector(
               "/odom",
               Odometry,
               key="odom"
           )

           self.spin_for_duration(3.0)

           messages = collector.get_messages()
           assert len(messages) > 0, "No odometry messages received"


   if __name__ == "__main__":
       pytest.main([__file__, "-v"])

Run the Test
------------

Start your robot simulation, then run:

.. code-block:: bash

   pytest test_my_robot.py -v

Understanding the Code
----------------------

Test Fixture
~~~~~~~~~~~~

.. code-block:: python

   class TestMyRobot(SimTestFixture):

``SimTestFixture`` is the base class that:

- Creates a ROS 2 node for your test
- Handles setup and teardown
- Provides helper methods

Message Collection
~~~~~~~~~~~~~~~~~~

.. code-block:: python

   collector = self.create_message_collector("/scan", LaserScan, key="scan")

This creates a subscriber that buffers messages. The ``key`` parameter lets you
retrieve the collector later if needed.

Spinning
~~~~~~~~

.. code-block:: python

   self.spin_for_duration(5.0)

This spins the ROS node for 5 seconds, allowing:

- Subscribers to receive messages
- Publishers to send data
- Callbacks to execute

Assertions
~~~~~~~~~~

.. code-block:: python

   assert len(messages) > 0, "No LIDAR messages received"

Use standard pytest assertions. The message after the comma is shown on failure.

Adding Requirements Tracking
-----------------------------

To track test results against requirement IDs, mix in ``RequirementValidator``:

.. code-block:: python

   from sim_harness import SimTestFixture, RequirementValidator

   class TestMyRobot(SimTestFixture, RequirementValidator):
       def test_lidar_publishes(self):
           collector = self.create_message_collector("/scan", LaserScan)
           self.spin_for_duration(5.0)
           messages = collector.get_messages()

           passed = len(messages) > 0

           self.assert_requirement(
               req_id="REQ-SEN-001",
               description="LIDAR sensor publishes scan data",
               passed=passed,
               details=f"Received {len(messages)} messages",
               category="Sensors"
           )

Next Steps
----------

- :doc:`../guide/fixtures` - Learn about fixture types
- :doc:`../guide/message_collection` - Advanced collection patterns
- :doc:`../guide/assertions` - Built-in assertion functions
