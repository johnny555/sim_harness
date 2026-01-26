Contributing
============

Thank you for your interest in contributing to sim_harness!

Getting Started
---------------

Fork and Clone
~~~~~~~~~~~~~~

.. code-block:: bash

   git clone https://github.com/YOUR_USERNAME/sim_harness.git
   cd sim_harness

Set Up Development Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   # Create virtual environment
   python3 -m venv venv
   source venv/bin/activate

   # Install development dependencies
   pip install -e ".[dev]"

   # Or install manually
   pip install pytest pytest-cov mypy ruff sphinx sphinx-rtd-theme

Build and Test
~~~~~~~~~~~~~~

.. code-block:: bash

   # Build the package
   cd ~/ros2_ws
   colcon build --packages-select sim_harness

   # Run tests
   pytest src/sim_harness/test/ -v

   # Run with coverage
   pytest src/sim_harness/test/ -v --cov=sim_harness --cov-report=html

Development Workflow
--------------------

1. Create a Branch
~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   git checkout -b feature/my-new-feature
   # or
   git checkout -b fix/issue-123

2. Make Changes
~~~~~~~~~~~~~~~

- Write code following the style guide (below)
- Add tests for new functionality
- Update documentation if needed

3. Run Checks
~~~~~~~~~~~~~

.. code-block:: bash

   # Format code
   ruff format sim_harness/

   # Lint
   ruff check sim_harness/

   # Type check
   mypy sim_harness/

   # Run tests
   pytest test/ -v

4. Commit
~~~~~~~~~

.. code-block:: bash

   git add .
   git commit -m "feat: add new assertion for battery state"

Follow `Conventional Commits <https://www.conventionalcommits.org/>`_:

- ``feat:`` - New feature
- ``fix:`` - Bug fix
- ``docs:`` - Documentation only
- ``test:`` - Adding tests
- ``refactor:`` - Code change that doesn't fix a bug or add a feature

5. Push and Create PR
~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   git push origin feature/my-new-feature

Then create a Pull Request on GitHub.

Code Style
----------

Python
~~~~~~

- Follow PEP 8
- Use type hints
- Maximum line length: 100 characters
- Use docstrings for public APIs

.. code-block:: python

   def assert_vehicle_moved(
       node: Node,
       vehicle_id: str,
       min_distance: float,
       timeout_sec: float = 10.0,
   ) -> MovementResult:
       """
       Assert that a vehicle moved at least min_distance meters.

       Args:
           node: ROS 2 node for subscriptions
           vehicle_id: Vehicle namespace (e.g., "robot_01")
           min_distance: Minimum distance to move (meters)
           timeout_sec: Maximum time to wait

       Returns:
           MovementResult with success status and details
       """
       ...

C++
~~~

- Follow ROS 2 C++ style guide
- Use clang-format

.. code-block:: cpp

   /**
    * @brief Assert that a vehicle moved at least min_distance meters.
    *
    * @param node ROS 2 node
    * @param vehicle_id Vehicle namespace
    * @param min_distance Minimum distance (meters)
    * @return MovementResult
    */
   MovementResult assertVehicleMoved(
       rclcpp::Node::SharedPtr node,
       const std::string& vehicle_id,
       double min_distance);

Adding New Features
-------------------

New Assertion Primitive
~~~~~~~~~~~~~~~~~~~~~~~

1. Create the function in ``sim_harness/primitives/``
2. Add tests in ``test/python/test_*.py``
3. Export in ``sim_harness/__init__.py``
4. Document in ``docs/api/primitives.rst``

Example:

.. code-block:: python

   # sim_harness/primitives/battery_assertions.py
   from dataclasses import dataclass

   @dataclass
   class BatteryResult:
       success: bool
       voltage: float
       details: str

   def assert_battery_voltage(
       node,
       topic: str,
       min_voltage: float,
       timeout_sec: float = 5.0
   ) -> BatteryResult:
       """Assert battery voltage is above minimum."""
       # Implementation
       ...

New Test Fixture
~~~~~~~~~~~~~~~~

1. Create class in ``sim_harness/core/``
2. Inherit from ``SimTestFixture`` or ``SimulationTestFixture``
3. Add tests
4. Document usage

Running Documentation Locally
-----------------------------

.. code-block:: bash

   # Install docs dependencies
   pip install sphinx sphinx-rtd-theme

   # Build docs
   cd docs
   sphinx-build -b html . _build/html

   # Open in browser
   xdg-open _build/html/index.html

Reporting Issues
----------------

When reporting issues, please include:

1. **Description** - What happened vs. what you expected
2. **Steps to reproduce** - Minimal example to reproduce
3. **Environment** - ROS version, OS, Python version
4. **Logs** - Relevant error messages or output

Questions?
----------

- Open a `Discussion <https://github.com/johnny555/sim_harness/discussions>`_ for questions
- Open an `Issue <https://github.com/johnny555/sim_harness/issues>`_ for bugs or feature requests

Thank you for contributing!
