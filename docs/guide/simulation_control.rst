Simulation Control
==================

sim_harness can automatically manage Gazebo simulation lifecycle.

Automatic Management
--------------------

.. code-block:: python

   class TestWithSim(SimulationTestFixture):
       LAUNCH_PACKAGE = 'turtlebot3_gazebo'
       LAUNCH_FILE = 'turtlebot3_world.launch.py'

       def test_robot_spawned(self):
           # Simulation is already running
           pass

Configuration
-------------

.. code-block:: python

   LAUNCH_ARGS = {'use_sim_time': 'true', 'world': 'my_world.sdf'}
   ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}
   STARTUP_TIMEOUT = 120.0
   GAZEBO_STARTUP_DELAY = 10.0

Using Pre-Started Simulation
----------------------------

For faster development:

.. code-block:: python

   class TestQuickIteration(SimulationTestFixture):
       USE_EXISTING_SIM = True
       REQUIRE_SIM = True

Headless Mode
-------------

.. code-block:: bash

   GAZEBO_HEADLESS=1 pytest tests/ -v
