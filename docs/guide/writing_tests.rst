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

   from sim_harness.nav2 import assert_lifecycle_node_active

   @pytest.mark.integration
   def test_navigation_stack(self):
       nodes = ["controller_server", "planner_server"]
       for node_name in nodes:
           result = assert_lifecycle_node_active(self.node, node_name)
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
