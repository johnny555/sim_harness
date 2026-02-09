sim_harness
===========

A ROS 2 simulation testing framework for robotics applications.

**Quick to Set Up** - **pytest Integration** - **Simulation Control** - **Requirements Tracking**

What is sim_harness?
--------------------

**sim_harness** is a testing framework designed for ROS 2 robotics applications
that run in simulation. It provides:

- **CLI Integration** - ``ros2 test`` command for test discovery and execution
- **Test Fixtures** - Base classes that handle ROS 2 node lifecycle
- **Message Collectors** - Easy subscription and message buffering
- **Simulation Control** - Automatic Gazebo lifecycle management
- **Assertion Library** - Pre-built checks for sensors, timing, motion, navigation, and perception
- **Requirements Tracking** - Opt-in requirement-to-test mapping for compliance reporting

Quick Example
-------------

.. code-block:: python

   from sim_harness import SimTestFixture
   from sensor_msgs.msg import LaserScan

   class TestMyRobot(SimTestFixture):
       def test_lidar_publishes(self):
           # Collect messages from a topic
           collector = self.create_message_collector("/scan", LaserScan)

           # Wait for messages
           self.spin_for_duration(3.0)

           # Assert we received data
           messages = collector.get_messages()
           assert len(messages) > 0, "No LIDAR messages received"
           assert len(messages[-1].ranges) > 100, "LIDAR should have > 100 ranges"

Run it with pytest:

.. code-block:: bash

   pytest test_my_robot.py -v

Features
--------

Simple Test Writing
~~~~~~~~~~~~~~~~~~~

Write tests using familiar pytest patterns. No complex setup required.

Automatic Simulation Lifecycle
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   class TestWithSim(SimTestFixture):
       LAUNCH_PACKAGE = 'my_robot_gazebo'
       LAUNCH_FILE = 'simulation.launch.py'

       def test_something(self):
           # Simulation is already running!
           pass

Built-in Assertions
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   from sim_harness import assert_lidar_valid, assert_vehicle_moved

   # Validate LIDAR data quality
   result = assert_lidar_valid(node, "/scan")
   assert result.valid, result.details

   # Check robot moved
   result = assert_vehicle_moved(node, min_distance=1.0)
   assert result.success, result.details

Nav2 and Perception Extensions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Specialized assertions live in dedicated extension modules:

.. code-block:: python

   from sim_harness.nav2 import assert_nav2_active, assert_reaches_goal
   from sim_harness.perception import assert_object_detected

   # Check all Nav2 nodes are active
   results = assert_nav2_active(node, timeout_sec=60.0)
   assert all(r.success for r in results)

Requirements Tracking (opt-in)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add requirement tracing by mixing in ``RequirementValidator``:

.. code-block:: python

   from sim_harness import SimTestFixture, RequirementValidator

   class TestWithReqs(SimTestFixture, RequirementValidator):
       def test_lidar_operational(self):
           # ... test code ...

           self.assert_requirement(
               "REQ-SEN-001",
               "LIDAR sensor publishes valid data",
               passed=True,
               details="Received 100 messages",
               category="Sensors"
           )

Installation
------------

.. code-block:: bash

   pip install sim-harness

Or from source:

.. code-block:: bash

   cd ~/ros2_ws/src
   git clone https://github.com/johnny555/sim_harness.git
   cd ..
   colcon build --packages-select sim_harness

License
-------

sim_harness is released under the Apache 2.0 License.

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   getting_started/installation
   getting_started/quickstart
   getting_started/first_test

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   guide/cli
   guide/writing_tests
   guide/fixtures
   guide/message_collection
   guide/simulation_control
   guide/assertions
   guide/requirements

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/core
   api/primitives
   api/simulation
   api/validation

.. toctree::
   :maxdepth: 2
   :caption: Examples

   examples/turtlebot3
   examples/custom_robot

.. toctree::
   :maxdepth: 1
   :caption: Development

   contributing
