Readiness Checks
================

sim_harness provides a configurable readiness check system for validating
that a robot (real or simulated) is ready for testing. This is useful for:

- Waiting for all nodes to start before running tests
- Validating hardware is connected and publishing
- Ensuring controllers are loaded and active
- Checking TF tree is complete

Basic Usage
-----------

.. code-block:: python

   from sim_harness import ReadinessCheck

   # Create a readiness check with a ROS node
   check = ReadinessCheck(node)

   # Add checks
   check.add_topic("/scan", LaserScan)
   check.add_topic("/odom", Odometry)
   check.add_node("robot_state_publisher")
   check.add_transform("odom", "base_link")

   # Run checks
   result = check.run()

   if result.ready:
       print("Robot ready!")
   else:
       print(f"Failed: {result.failed_checks()}")

Fluent API
----------

Checks can be chained for readability:

.. code-block:: python

   result = (
       ReadinessCheck(node)
       .add_topic("/scan", LaserScan)
       .add_topic("/odom", Odometry)
       .add_topic("/cmd_vel", Twist)
       .add_node("robot_state_publisher")
       .add_service("/get_state")
       .add_transform("odom", "base_link")
       .add_transform("base_link", "laser")
       .run(verbose=True)
   )

   assert result.ready, f"Robot not ready: {result.summary()}"

Check Types
-----------

Topics
^^^^^^

Check that topics are being published:

.. code-block:: python

   # Basic topic check
   check.add_topic("/scan", LaserScan)

   # With custom timeout
   check.add_topic("/camera/image", Image, timeout_sec=10.0)

   # Critical topics (fail fast)
   check.add_topic("/odom", Odometry, required=True)

Nodes
^^^^^

Check that nodes are running:

.. code-block:: python

   # Single node
   check.add_node("robot_state_publisher")

   # With timeout
   check.add_node("controller_manager", timeout_sec=15.0)

Services
^^^^^^^^

Check that services are available:

.. code-block:: python

   check.add_service("/get_state")
   check.add_service("/controller_manager/list_controllers")

Transforms
^^^^^^^^^^

Check that TF transforms are available:

.. code-block:: python

   check.add_transform("odom", "base_link")
   check.add_transform("map", "odom", timeout_sec=30.0)

Lifecycle Nodes
^^^^^^^^^^^^^^^

Check lifecycle node states:

.. code-block:: python

   # Check node is active
   check.add_lifecycle_node("sensor_node")

   # Check specific state
   check.add_lifecycle_node("recorder_node", expected_state="inactive")

Controllers
^^^^^^^^^^^

Check ros2_control controllers:

.. code-block:: python

   check.add_controller("joint_state_broadcaster")
   check.add_controller("diff_drive_controller", expected_state="active")

Custom Checks
^^^^^^^^^^^^^

Add custom validation logic:

.. code-block:: python

   def check_battery_level():
       # Your custom logic here
       return battery_level > 20.0

   check.add_custom("battery_ok", check_battery_level,
                    description="Battery level above 20%")

Standard Robot Checks
---------------------

Use ``create_standard_check`` for common robot configurations:

.. code-block:: python

   from sim_harness import create_standard_check

   # Create check for a differential drive robot
   check = create_standard_check(
       node,
       sensors=["/scan", "/odom", "/imu"],
       controllers=["joint_state_broadcaster", "diff_drive_controller"],
       tf_frames=[("odom", "base_link"), ("base_link", "laser")],
   )

   result = check.run(verbose=True)

Check Results
-------------

The ``CheckResult`` object provides detailed information:

.. code-block:: python

   result = check.run()

   # Overall status
   if result.ready:
       print("All checks passed")

   # Summary
   print(result.summary())
   # Output: "8/10 checks passed"

   # Detailed results (import CheckStatus for comparison)
   from sim_harness import CheckStatus
   for item in result.items:
       status = "PASS" if item.status == CheckStatus.PASSED else "FAIL"
       print(f"[{status}] {item.name}: {item.message}")

   # Failed items only
   for item in result.failed_checks():
       print(f"FAILED: {item.name} - {item.message}")

   # Duration
   print(f"Checks completed in {result.duration_sec:.2f}s")

Verbose Output
--------------

Enable verbose mode for real-time progress:

.. code-block:: python

   result = check.run(verbose=True)

Output::

   [CHECK] Checking topic /scan... OK (0.12s)
   [CHECK] Checking topic /odom... OK (0.08s)
   [CHECK] Checking node robot_state_publisher... OK (0.05s)
   [CHECK] Checking transform odom->base_link... OK (0.03s)
   [CHECK] Checking controller diff_drive_controller... FAIL (5.00s)

   Readiness Check Results:
     PASSED: 4
     FAILED: 1
     Total time: 5.28s

Integration with Test Fixtures
------------------------------

Use readiness checks in test setup:

.. code-block:: python

   from sim_harness import SimTestFixture, ReadinessCheck

   class TestNavigation(SimTestFixture):

       def setup_method(self):
           super().setup_method()

           # Wait for robot to be ready
           result = (
               ReadinessCheck(self.node)
               .add_topic("/scan", LaserScan)
               .add_topic("/odom", Odometry)
               .add_lifecycle_node("bt_navigator", expected_state="active")
               .add_transform("map", "base_link")
               .run(verbose=True)
           )

           assert result.ready, f"Robot not ready: {result.summary()}"

Real Robot Testing
------------------

Readiness checks are particularly useful for real robot testing where
startup timing is unpredictable:

.. code-block:: python

   class TestRealRobot(SimTestFixture):
       """Tests for physical robot hardware."""

       def setup_method(self):
           super().setup_method()

           # Wait for hardware to be ready
           result = (
               ReadinessCheck(self.node, default_timeout_sec=30.0)
               .add_topic("/joint_states", JointState, required=True)
               .add_topic("/scan", LaserScan, required=True)
               .add_topic("/odom", Odometry)
               .add_controller("joint_state_broadcaster")
               .add_controller("diff_drive_controller")
               .add_transform("odom", "base_footprint")
               .add_custom("hardware_enabled", self._check_hardware_enabled)
               .run(verbose=True)
           )

           if not result.ready:
               pytest.skip(f"Hardware not ready: {result.summary()}")

       def _check_hardware_enabled(self):
           # Custom hardware check
           return self._hardware_enabled

Categories and Filtering
------------------------

Checks are organized by category for easier analysis:

.. code-block:: python

   from sim_harness import CheckCategory

   result = check.run()

   # Get results by category
   topic_results = [i for i in result.items
                    if i.category == CheckCategory.TOPIC]
   tf_results = [i for i in result.items
                 if i.category == CheckCategory.TRANSFORM]

   # Check if specific category passed
   all_topics_ok = all(i.success for i in topic_results)

Error Handling
--------------

Checks handle errors gracefully:

.. code-block:: python

   # Checks that fail return False, not exceptions
   result = check.run()

   for item in result.failed_checks():
       print(f"{item.name}: {item.message}")
       # Output: "topic:/scan: No messages received within 5.0s"

Timeouts
--------

Configure timeouts at different levels:

.. code-block:: python

   # Default timeout for all checks
   check = ReadinessCheck(node, default_timeout_sec=10.0)

   # Per-check timeout (overrides default)
   check.add_topic("/slow_topic", SlowMsg, timeout_sec=30.0)

   # Critical checks fail immediately on timeout
   check.add_topic("/critical", CriticalMsg, required=True)

Best Practices
--------------

1. **Start with essential checks**: Only check what's needed for your tests
2. **Use appropriate timeouts**: Hardware may take longer than simulation
3. **Mark critical items**: Use ``required=True`` for must-have dependencies
4. **Enable verbose in CI**: Helps debug failures in automated pipelines
5. **Consider skip vs fail**: Use ``pytest.skip()`` for optional hardware

.. code-block:: python

   # Good: Focused checks for navigation test
   check = (
       ReadinessCheck(node)
       .add_topic("/scan", LaserScan)
       .add_lifecycle_node("bt_navigator", expected_state="active")
       .add_transform("map", "base_link")
   )

   # Avoid: Checking everything when you only need sensors
   # check.add_controller("arm_controller")  # Not needed for nav test
