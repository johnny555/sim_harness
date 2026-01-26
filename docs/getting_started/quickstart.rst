Quick Start
===========

This guide will get you running tests quickly.

Run the Example Tests
---------------------

sim_harness comes with example tests for TurtleBot3. Let's run them.

1. Start the Simulation
~~~~~~~~~~~~~~~~~~~~~~~

In one terminal, start the TurtleBot3 simulation:

.. code-block:: bash

   export TURTLEBOT3_MODEL=waffle
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2. Run the Tests
~~~~~~~~~~~~~~~~

In another terminal, run the example tests:

.. code-block:: bash

   cd ~/ros2_ws/src/sim_harness
   pytest examples/test_turtlebot3_integration.py -v -k "Basic"

You should see output like:

.. code-block:: text

   examples/test_turtlebot3_integration.py::TestTurtleBot3Integration::test_basic_lidar_publishes PASSED
   examples/test_turtlebot3_integration.py::TestTurtleBot3Integration::test_basic_odometry_publishes PASSED
   examples/test_turtlebot3_integration.py::TestTurtleBot3Integration::test_basic_imu_publishes PASSED

Run Tests with Auto-Simulation
------------------------------

sim_harness can automatically start and stop the simulation:

.. code-block:: bash

   pytest examples/test_turtlebot3_with_sim_control.py -v -s

.. note::

   The ``-s`` flag shows simulation startup output. The first run may take
   longer as Gazebo initializes.

Using the CLI
-------------

sim_harness provides a ``ros2 test`` command for test discovery and execution:

.. code-block:: bash

   # List all tests in your workspace
   ros2 test list

   # List tests with details
   ros2 test list -v

   # Run all tests
   ros2 test run

   # Run a specific test
   ros2 test run test_turtlebot3_integration

   # Run tests matching a pattern
   ros2 test run --pattern "test_basic*"

   # Run tests from a specific package
   ros2 test run -p my_robot_tests

   # Rerun failed tests
   ros2 test failed

See :doc:`../guide/cli` for the complete CLI reference.

What's Next?
------------

- :doc:`first_test` - Write your own test
- :doc:`../guide/writing_tests` - Learn the test patterns
- :doc:`../guide/simulation_control` - Auto-start simulations
