Simulation API
==============

Classes for managing Gazebo simulation lifecycle across test sessions.

Overview
--------

The simulation API provides two main components that work together:

- **SimulationRequest**: A data class describing *what* simulation configuration you need
- **SimulationManager**: A singleton service that manages *how* simulations run

The key insight is that Gazebo simulations are expensive to start (10-30 seconds),
so the manager intelligently reuses running simulations when possible.

SimulationRequest vs SimulationManager
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 20 40 40

   * - Aspect
     - SimulationRequest
     - SimulationManager
   * - **Purpose**
     - Describes a simulation configuration
     - Controls simulation lifecycle
   * - **Pattern**
     - Data Transfer Object (DTO)
     - Singleton service
   * - **Instances**
     - One per test/fixture
     - One per process (shared)
   * - **Contains**
     - Package, launch file, args, world, robot
     - Launcher, current state, user count
   * - **Key method**
     - ``config_hash()`` - identity check
     - ``request()`` - start or reuse simulation

How They Work Together
----------------------

When a test needs a simulation:

1. **Test creates a SimulationRequest** describing what it needs::

       request = SimulationRequest(
           package='turtlebot3_gazebo',
           launch_file='turtlebot3_world.launch.py',
           world='turtlebot3_world',
       )

2. **Test calls SimulationManager.request()** with the request::

       manager = SimulationManager.get_instance()
       manager.request(request)

3. **Manager decides what to do** based on current state:

   - **No simulation running**: Start a new one
   - **Compatible simulation running**: Reuse it (fast!)
   - **Incompatible simulation running**: Stop it, start new one

4. **Compatibility is determined by config_hash()**: Two requests are compatible
   if they have the same package, launch file, world, robot, and key launch args.

Lifecycle Diagram
-----------------

::

    Test A                    SimulationManager                 Gazebo
      │                             │                             │
      │─── request(configA) ───────>│                             │
      │                             │────── start() ─────────────>│
      │                             │<───── running ──────────────│
      │<─── ready ──────────────────│                             │
      │                             │                             │
      │    (test runs)              │                             │
      │                             │                             │
      │─── release() ──────────────>│  (sim stays running)        │
      │                             │                             │
    Test B                          │                             │
      │                             │                             │
      │─── request(configA) ───────>│  (same hash = reuse!)       │
      │<─── ready (instant!) ───────│                             │
      │                             │                             │
    Test C                          │                             │
      │                             │                             │
      │─── request(configB) ───────>│  (different hash)           │
      │                             │────── stop() ──────────────>│
      │                             │────── start(configB) ──────>│
      │<─── ready ──────────────────│                             │


SimulationRequest
-----------------

A dataclass that captures all the information needed to start a simulation.

.. autoclass:: sim_harness.simulator.simulation_manager.SimulationRequest
   :members:
   :undoc-members:
   :show-inheritance:

**Key Fields:**

``package``
    The ROS 2 package containing the launch file (e.g., ``'turtlebot3_gazebo'``).

``launch_file``
    The launch file name (e.g., ``'turtlebot3_world.launch.py'``).

``launch_args``
    Dictionary of launch arguments passed to the launch file.

``world``
    World file or name. Used in ``config_hash()`` to detect when a restart is needed.

``robot_model``
    Robot model name. Also used in ``config_hash()`` for restart detection.

**Config Hash:**

The ``config_hash()`` method generates a short hash from the key configuration
values. Two requests with the same hash are considered "compatible" and the
manager will reuse the running simulation instead of restarting.

These launch args trigger a restart when changed:
``world``, ``world_name``, ``robot_model``, ``model``, ``vehicle``, ``headless``, ``gui``

SimulationManager
-----------------

The singleton manager that coordinates Gazebo across all tests.

.. autoclass:: sim_harness.simulator.simulation_manager.SimulationManager
   :members:
   :undoc-members:
   :show-inheritance:

**Key Methods:**

``get_instance()``
    Class method to get the singleton instance. Thread-safe.

``request(request, startup_timeout, gazebo_delay, require_sim)``
    Request a simulation. Starts, reuses, or restarts as needed.
    Increments the active user count.

``release()``
    Release the simulation (decrements user count). Does NOT stop it.
    This allows subsequent tests to reuse the simulation.

``stop(force=False)``
    Actually stop the simulation. Only stops if no active users,
    unless ``force=True``.

``restart(request=None)``
    Force restart the simulation, optionally with a new configuration.

``is_running()``
    Check if Gazebo is currently running.

**User Counting:**

The manager tracks how many tests are currently using the simulation via
``active_users``. This enables:

- ``release()`` to leave the simulation running for the next test
- ``stop()`` to wait until all tests are done (unless forced)
- Proper cleanup on process exit via ``atexit``

Convenience Functions
---------------------

For simple cases, you can use these module-level functions:

.. autofunction:: sim_harness.simulator.simulation_manager.get_simulation_manager

The ``SimulationTestFixture`` base class handles all of this automatically,
so you typically don't need to call the manager directly.

Usage Examples
--------------

**Basic usage (manual):**

.. code-block:: python

   from sim_harness.simulator import SimulationManager, SimulationRequest

   # Get the singleton manager
   manager = SimulationManager.get_instance()

   # Create a request
   request = SimulationRequest(
       package='my_robot_gazebo',
       launch_file='simulation.launch.py',
       launch_args={'use_sim_time': 'true'},
       world='warehouse',
   )

   # Request the simulation (starts if needed)
   manager.request(request, startup_timeout=60.0)

   # ... run your test ...

   # Release (allows reuse by next test)
   manager.release()

**Using the fixture (recommended):**

.. code-block:: python

   from sim_harness import SimulationTestFixture

   class TestMyRobot(SimulationTestFixture):
       # These become part of the SimulationRequest
       LAUNCH_PACKAGE = 'my_robot_gazebo'
       LAUNCH_FILE = 'simulation.launch.py'
       WORLD = 'warehouse'

       def test_something(self):
           # Simulation is already running
           pass

**Checking if restart is needed:**

.. code-block:: python

   request1 = SimulationRequest(package='pkg', launch_file='sim.launch.py', world='world_a')
   request2 = SimulationRequest(package='pkg', launch_file='sim.launch.py', world='world_a')
   request3 = SimulationRequest(package='pkg', launch_file='sim.launch.py', world='world_b')

   # Same configuration
   assert request1.config_hash() == request2.config_hash()

   # Different world = different hash = restart needed
   assert request1.config_hash() != request3.config_hash()
