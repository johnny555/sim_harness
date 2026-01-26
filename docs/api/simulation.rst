Simulation API
==============

Classes for managing Gazebo simulation lifecycle.

SimulationManager
-----------------

Singleton manager for simulation lifecycle.

.. code-block:: python

   from sim_harness.simulator import SimulationManager

   manager = SimulationManager.get_instance()
   success = manager.request(request, startup_timeout=60.0)
   manager.release()
   manager.stop()

SimulationRequest
-----------------

Configuration for simulation request.

.. code-block:: python

   from sim_harness.simulator import SimulationRequest

   request = SimulationRequest(
       package='my_robot_gazebo',
       launch_file='simulation.launch.py',
       launch_args={'use_sim_time': 'true'},
   )

Convenience Functions
---------------------

- ``start_simulation(package, launch_file, timeout)``
- ``stop_simulation()``
- ``is_simulation_running()``
