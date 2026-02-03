Test Fixtures
=============

sim_harness provides test fixtures that handle ROS 2 node lifecycle.

SimTestFixture
--------------

The main fixture for all simulation tests. Provides a ROS 2 node, executor,
message collectors, spin helpers, and optional simulation management.

.. code-block:: python

   from sim_harness import SimTestFixture

   class TestMyRobot(SimTestFixture):
       def test_something(self):
           # self.node is available
           # self.executor is available
           pass

Simulation Control
~~~~~~~~~~~~~~~~~~

Set class attributes to automatically start/stop a Gazebo simulation:

.. code-block:: python

   class TestWithSim(SimTestFixture):
       LAUNCH_PACKAGE = 'turtlebot3_gazebo'
       LAUNCH_FILE = 'turtlebot3_world.launch.py'
       LAUNCH_ARGS = {'use_sim_time': 'true'}
       STARTUP_TIMEOUT = 60.0
       GAZEBO_STARTUP_DELAY = 5.0
       REQUIRE_SIM = True
       USE_EXISTING_SIM = False

       def test_something(self):
           # Simulation is already running!
           pass

``SimulationTestFixture`` is an alias for ``SimTestFixture`` (backwards compatibility).

Requirements Tracking (opt-in)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To add requirements tracking, mix in ``RequirementValidator``:

.. code-block:: python

   from sim_harness import SimTestFixture, RequirementValidator

   class TestWithReqs(SimTestFixture, RequirementValidator):
       def test_sensor(self):
           # self.assert_requirement(...) is now available
           self.assert_requirement(
               "REQ-SEN-001", "LIDAR publishes data",
               True, "Received 100 messages", "Sensors"
           )

Custom Setup/Teardown
~~~~~~~~~~~~~~~~~~~~~

Override ``on_setup()`` and ``on_teardown()`` for custom lifecycle hooks:

.. code-block:: python

   class TestMyRobot(SimTestFixture):
       def on_setup(self):
           self.scan = self.create_message_collector("/scan", LaserScan)
           self.odom = self.create_message_collector("/odom", Odometry)

       def on_teardown(self):
           # Custom cleanup if needed
           pass

Standalone Fixtures
-------------------

For tests that don't need a full class-based fixture:

.. code-block:: python

   def test_something(ros_node, ros_executor):
       # ros_node is a plain ROS 2 node with use_sim_time=True
       # ros_executor is a SingleThreadedExecutor with the node attached
       pass
