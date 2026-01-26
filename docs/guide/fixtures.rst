Test Fixtures
=============

sim_harness provides test fixtures that handle ROS 2 node lifecycle.

SimTestFixture
--------------

Basic fixture for tests where simulation is already running:

.. code-block:: python

   from sim_harness import SimTestFixture

   class TestMyRobot(SimTestFixture):
       def test_something(self):
           # self.node is available
           pass

SimulationTestFixture
---------------------

Automatically starts and stops simulation:

.. code-block:: python

   from sim_harness import SimulationTestFixture

   class TestWithSim(SimulationTestFixture):
       LAUNCH_PACKAGE = 'turtlebot3_gazebo'
       LAUNCH_FILE = 'turtlebot3_world.launch.py'

       def test_something(self):
           # Simulation is already running!
           pass

Configuration Options
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: python

   class TestWithSim(SimulationTestFixture):
       LAUNCH_PACKAGE = 'my_robot_gazebo'
       LAUNCH_FILE = 'simulation.launch.py'
       LAUNCH_ARGS = {'use_sim_time': 'true'}
       ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}
       STARTUP_TIMEOUT = 60.0
       GAZEBO_STARTUP_DELAY = 5.0
       REQUIRE_SIM = True
       USE_EXISTING_SIM = False
