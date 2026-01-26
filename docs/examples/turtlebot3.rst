TurtleBot3 Integration
======================

Example tests for TurtleBot3 robots.

Basic Tests
-----------

.. code-block:: python

   from sim_harness import SimTestFixture

   class TestTurtleBot3Basic(SimTestFixture):
       def test_lidar_publishes(self):
           collector = self.create_message_collector("/scan", LaserScan)
           self.spin_for_duration(3.0)
           assert collector.message_count() > 0

Automatic Simulation
--------------------

.. code-block:: python

   from sim_harness import SimulationTestFixture

   class TestTurtleBot3Auto(SimulationTestFixture):
       LAUNCH_PACKAGE = 'turtlebot3_gazebo'
       LAUNCH_FILE = 'turtlebot3_world.launch.py'
       ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}

       def test_sensors(self):
           # Simulation already running
           pass

Movement Testing
----------------

.. code-block:: python

   def test_robot_moves_forward(self):
       odom = self.create_message_collector("/odom", Odometry)
       cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
       # Send commands and verify movement
