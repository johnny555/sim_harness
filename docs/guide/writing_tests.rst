Writing Tests
=============

This guide covers the patterns and best practices for writing sim_harness tests.

Test Structure
--------------

A typical test file follows this structure:

.. code-block:: python

   #!/usr/bin/env python3
   """Description of what this test file covers."""

   import pytest
   from sensor_msgs.msg import LaserScan

   from sim_harness import SimTestFixture


   class TestSensorIntegration(SimTestFixture):
       """Test class docstring - describes the test suite."""

       SCAN_TOPIC = "/scan"

       def test_something(self):
           """Test method docstring - describes this specific test."""
           collector = self.create_message_collector(self.SCAN_TOPIC, LaserScan)
           self.spin_for_duration(3.0)
           messages = collector.get_messages()
           assert len(messages) > 0

Test Categories
---------------

**Basic Tests** - Verify fundamental functionality:

.. code-block:: python

   def test_basic_topic_publishes(self):
       collector = self.create_message_collector("/scan", LaserScan)
       self.spin_for_duration(3.0)
       assert len(collector.get_messages()) > 0

**Integration Tests** - Verify multiple components:

.. code-block:: python

   @pytest.mark.integration
   def test_navigation_stack(self):
       nodes = ["controller_server", "planner_server"]
       for node in nodes:
           result = assert_lifecycle_node_active(self.node, node)
           assert result.success

**Motion Tests** - Involve robot movement:

.. code-block:: python

   def test_robot_moves_forward(self):
       odom = self.create_message_collector("/odom", Odometry)
       self.spin_for_duration(1.0)
       initial_x = odom.get_messages()[-1].pose.pose.position.x
       # Send velocity commands...
       # Check distance moved

Using Markers
-------------

.. code-block:: bash

   pytest -m integration      # Run only integration tests
   pytest -m "not slow"       # Skip slow tests
   pytest -k "basic"          # Run basic tests only

Best Practices
--------------

.. tip::

   Keep tests focused - each test should verify one thing.

.. tip::

   Use descriptive names - ``test_lidar_publishes_at_10hz`` is better than ``test_lidar``.

.. warning::

   Avoid test interdependence - tests should not depend on other tests running first.

Property-Based Testing
----------------------

sim_harness integrates with `Hypothesis <https://hypothesis.readthedocs.io/>`_
for property-based testing of robotic systems.  Instead of writing individual
test cases with specific values, you describe *properties* that must hold and
let Hypothesis generate test inputs automatically.

**Why property-based testing for robotics?**

- Finds edge cases you wouldn't think to test manually
- When a test fails, Hypothesis *shrinks* to the simplest failing input
- Persistent database remembers failures across runs

**Three tiers** based on cost:

**Tier 1 — Properties over Recorded Data (cheap)**

Collect data once from the sim, then check many properties against the
recorded messages.  No sim re-run.  Full Hypothesis power.

.. code-block:: python

   from sim_harness.core.sim_property import (
       check_recorded_property,
       check_recorded_monotonic,
   )

   class TestSensorQuality(SimTestFixture):
       def test_lidar_quality(self):
           collector = self.create_message_collector("/scan", LaserScan)
           self.spin_for_duration(10.0)
           messages = collector.get_messages()

           # Every scan must have >= 100 valid points
           check_recorded_property(
               messages,
               lambda scan: sum(1 for r in scan.ranges if math.isfinite(r)) >= 100,
               description="All scans have >= 100 valid points",
           )

           # Timestamps must be non-decreasing
           check_recorded_monotonic(
               messages,
               extract=lambda s: s.header.stamp.sec + s.header.stamp.nanosec * 1e-9,
               description="Timestamps are non-decreasing",
           )

**Tier 2 — Scenario-Level (expensive, use sparingly)**

Hypothesis generates entire test scenarios.  Use ``sim_property`` with
``max_examples=3-5``.

.. code-block:: python

   from hypothesis import given
   from sim_harness.core.sim_property import sim_property
   from sim_harness.core.strategies import navigation_goal_2d

   class TestNavigation(SimTestFixture):
       @sim_property(max_examples=3)
       @given(goal=navigation_goal_2d(x_bounds=(-2, 2), y_bounds=(-2, 2)))
       def test_reaches_random_goals(self, goal):
           """Robot can navigate to any reachable goal."""
           # ... send goal, wait, check arrival

**Tier 3 — Same-Sim Variation (medium cost)**

The sim stays running.  Hypothesis varies commands between examples.

.. code-block:: python

   from sim_harness.core.strategies import twist_strategy

   class TestStability(SimTestFixture):
       @sim_property(max_examples=10)
       @given(cmd=twist_strategy(max_linear=0.3, max_angular=0.5))
       def test_any_twist_is_stable(self, cmd):
           """No velocity command should cause NaN in odometry."""
           # ... send cmd, observe, check no NaN

**Nightly mode**: Set ``SIM_HARNESS_NIGHTLY=1`` to run 10x more examples
for thorough overnight testing.

**Install**: ``pip install sim-harness[hypothesis]`` or add
``python3-hypothesis`` to your ``package.xml``.

See ``examples/test_property_based.py`` for complete examples and
:doc:`../api/core` for the full API reference.

FP-Inspired Stream Properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For checking properties over live message streams (as opposed to recorded data),
sim_harness provides Hedgehog-inspired combinators:

.. code-block:: python

   from sim_harness.core.stream_properties import for_all_messages, all_of
   from sim_harness.core.predicates import (
       scan_has_min_points, scan_ranges_within, scan_nan_ratio_below,
   )

   # Compose a predicate from small pieces
   valid_lidar = all_of(
       scan_has_min_points(100),
       scan_ranges_within(0.1, 30.0),
       scan_nan_ratio_below(0.05),
   )

   # Check it against every message on the topic
   result = for_all_messages(
       self.node, self.executor, "/scan", LaserScan,
       predicate=valid_lidar,
       timeout_sec=5.0,
       description="LIDAR data quality",
   )
   assert result.passed, result.counterexample_details

Available stream properties: ``for_all_messages``, ``eventually``, ``monotonic``.
Available combinators: ``all_of``, ``any_of``, ``negate``.

C++ Unit Tests (GTest/rtest)
----------------------------

For C++ components, you can write unit tests using GTest or rtest. These tests
are automatically discovered and can be run with ``ros2 test run --unit``.

**Basic GTest Example:**

.. code-block:: cpp

   // test/test_kinematics.cpp
   #include <gtest/gtest.h>
   #include "my_package/kinematics.hpp"

   TEST(KinematicsTest, NormalizeAngle) {
       EXPECT_DOUBLE_EQ(normalize_angle(0.0), 0.0);
       EXPECT_DOUBLE_EQ(normalize_angle(M_PI), M_PI);
       EXPECT_NEAR(normalize_angle(3 * M_PI), M_PI, 1e-10);
   }

   TEST(KinematicsTest, ComputeTrajectory) {
       auto trajectory = compute_trajectory(start, goal);
       EXPECT_FALSE(trajectory.empty());
       EXPECT_NEAR(trajectory.back().x, goal.x, 0.01);
   }

   int main(int argc, char** argv) {
       testing::InitGoogleTest(&argc, argv);
       return RUN_ALL_TESTS();
   }

**CMakeLists.txt Setup:**

.. code-block:: cmake

   # Add GTest dependency
   find_package(ament_cmake_gtest REQUIRED)

   # Create test executable with naming convention
   ament_add_gtest(test_kinematics test/test_kinematics.cpp)
   target_link_libraries(test_kinematics ${PROJECT_NAME})

   # Or for rtest
   ament_add_gtest(test_kinematics_rtest test/test_kinematics_rtest.cpp)

**Naming Conventions:**

For automatic discovery, name your test executables with one of these patterns:

- ``test_<name>`` (recommended)
- ``<name>_test``
- ``<name>_gtest``
- ``<name>_rtest``

**Running C++ Tests:**

.. code-block:: bash

   # Run all C++ unit tests
   ros2 test run --unit

   # Run a specific test
   ros2 test run test_kinematics

   # Run with verbose output to see individual test cases
   ros2 test run test_kinematics -v

**rtest for ROS 2 Integration:**

rtest provides additional utilities for ROS 2 testing:

.. code-block:: cpp

   #include <rtest/rtest.hpp>
   #include <rclcpp/rclcpp.hpp>

   class NodeTest : public rtest::RTestFixture {
   protected:
       void SetUp() override {
           node_ = std::make_shared<rclcpp::Node>("test_node");
       }

       rclcpp::Node::SharedPtr node_;
   };

   TEST_F(NodeTest, PublisherCreation) {
       auto pub = node_->create_publisher<std_msgs::msg::String>("topic", 10);
       EXPECT_NE(pub, nullptr);
   }

**When to Use Which Test Type:**

.. list-table::
   :widths: 30 35 35
   :header-rows: 1

   * - Test Type
     - Use For
     - Example
   * - GTest (``--unit``)
     - Pure C++ logic, algorithms, math
     - Kinematics, path planning algorithms
   * - rtest (``--unit``)
     - C++ code with ROS 2 dependencies
     - Node creation, message handling
   * - Launch test (``--integration``)
     - Full system integration, simulation
     - Navigation stack, sensor simulation
