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

Readiness and Cleanup
---------------------

Simulation startup fails closed by default. A Gazebo process must be running
and responsive via ``/clock`` before the harness reports the simulation as
ready. The compatibility option
``LaunchConfig(allow_process_only_ready=True)`` allows process-only readiness
for legacy flows, but new tests should keep the strict default.

Teardown is scoped to the ``ros2 launch`` process group started by the
harness. Broad pattern-based cleanup of Gazebo and ROS processes is available
only through explicit opt-in APIs such as
``launcher.stop(allow_global_cleanup=True)`` or
``manager.stop(allow_global_cleanup=True)``. Global cleanup can kill unrelated
ROS/Gazebo processes owned by the current user, so reserve it for manual
recovery or isolated CI jobs.

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

Gazebo Ground Truth
-------------------

sim_harness provides direct access to Gazebo's ground truth poses via
gz-transport, bypassing ROS 2 topics entirely. This is useful for:

- Verifying the robot actually moved in simulation
- Validating odometry accuracy against ground truth
- Getting precise positions without sensor noise or drift
- Debugging physics and collision issues

**Basic Usage:**

.. code-block:: python

   from sim_harness.simulator import GazeboGroundTruth

   with GazeboGroundTruth(world_name="my_world") as gz:
       # Get model pose
       pose = gz.get_model_pose("my_robot")
       if pose:
           print(f"Position: ({pose.x:.2f}, {pose.y:.2f}, {pose.z:.2f})")
           print(f"Orientation: yaw={pose.yaw:.2f} rad")

       # List all models
       print(f"Models in world: {gz.list_models()}")

**Comparing Odometry to Ground Truth:**

.. code-block:: python

   from sim_harness.simulator import GazeboGroundTruth

   with GazeboGroundTruth(world_name="turtlebot3_world") as gz:
       # Get ground truth
       gt = gz.get_model_pose("turtlebot3_waffle")

       # Compare with odometry (from ROS topic)
       odom_pos = (odom_msg.pose.pose.position.x,
                   odom_msg.pose.pose.position.y,
                   odom_msg.pose.pose.position.z)

       result = gz.compare_odom_to_ground_truth(
           "turtlebot3_waffle",
           odom_position=odom_pos,
           position_tolerance=0.5  # meters
       )

       if result.success:
           print(f"Odom accurate within {result.position_error:.3f}m")
       else:
           print(f"Odom drift detected: {result.position_error:.3f}m")

**One-Shot Pose Retrieval:**

For quick checks without maintaining a connection:

.. code-block:: python

   from sim_harness.simulator import get_model_pose_once

   pose = get_model_pose_once("my_robot", world_name="empty")
   if pose:
       print(f"Robot at: {pose.position}")

**Pose3D Data Class:**

The ``Pose3D`` class provides convenient access to position and orientation:

.. code-block:: python

   pose = gz.get_model_pose("robot")

   # Position
   x, y, z = pose.position  # or pose.x, pose.y, pose.z

   # Orientation (Euler angles)
   roll, pitch, yaw = pose.orientation

   # Distance calculations
   other_pose = gz.get_model_pose("goal")
   distance = pose.distance_to(other_pose)      # 3D distance
   distance_2d = pose.distance_2d_to(other_pose)  # XY plane only

**Requirements:**

Ground truth access requires the gz-transport Python bindings:

.. code-block:: bash

   # Usually installed with Gazebo Harmonic
   apt install python3-gz-transport13

You can check availability in code:

.. code-block:: python

   from sim_harness.simulator import GZ_TRANSPORT_AVAILABLE

   if GZ_TRANSPORT_AVAILABLE:
       # Use ground truth features
       pass
   else:
       # Fall back to ROS-only validation
       pass

**World Name:**

The world name is used to construct the Gazebo pose topic
(``/world/{world_name}/pose/info``). Common world names:

- ``empty`` - Default empty world
- ``turtlebot3_world`` - TurtleBot3 default
- Check your launch file for the world name used

**Integration with Assertions:**

The easiest way to use ground truth is via the vehicle assertions:

.. code-block:: python

   from sim_harness import assert_vehicle_moved_with_ground_truth

   result = assert_vehicle_moved_with_ground_truth(
       node,
       vehicle_id="robot",
       gazebo_model_name="my_robot_model",
       min_distance=1.0,
       world_name="my_world"
   )

   # Validates movement AND odom accuracy
   assert result.success
   assert result.odom_error < 0.1  # meters

See :doc:`assertions` for more details on ground truth validation in assertions.
