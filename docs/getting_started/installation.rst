Installation
============

Prerequisites
-------------

- **ROS 2** Humble or later
- **Python** 3.10+
- **Gazebo** Harmonic (for simulation tests)

Install from PyPI
-----------------

.. code-block:: bash

   pip install sim-harness

Install from Source
-------------------

Clone the repository into your ROS 2 workspace:

.. code-block:: bash

   cd ~/ros2_ws/src
   git clone https://github.com/johnny555/sim_harness.git

Build with colcon:

.. code-block:: bash

   cd ~/ros2_ws
   colcon build --packages-select sim_harness
   source install/setup.bash

Verify Installation
-------------------

Check that sim_harness is installed:

.. code-block:: bash

   python3 -c "import sim_harness; print(sim_harness.__version__)"

List available test commands:

.. code-block:: bash

   ros2 run sim_harness list_tests --help

Optional Dependencies
---------------------

For running the TurtleBot3 examples:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo

For development (docs, linting):

.. code-block:: bash

   pip install sphinx sphinx-rtd-theme pytest-cov

Next Steps
----------

- :doc:`quickstart` - Run your first test
- :doc:`first_test` - Write a custom test
