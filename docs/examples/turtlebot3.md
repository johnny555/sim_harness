# TurtleBot3 Integration Example

This example demonstrates testing a TurtleBot3 robot in Gazebo simulation.

## Prerequisites

Install TurtleBot3 packages:

```bash
sudo apt install ros-${ROS_DISTRO}-turtlebot3-gazebo
```

Set the robot model:

```bash
export TURTLEBOT3_MODEL=waffle
```

## Basic Test (Manual Simulation)

This test requires the simulation to be running externally.

### Start the Simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Run the Test

```python
#!/usr/bin/env python3
"""Basic TurtleBot3 test - requires simulation running."""

import pytest
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from sim_harness import SimTestFixture


class TestTurtleBot3Basic(SimTestFixture):
    """Basic tests that verify simulation is working."""

    def test_lidar_publishes(self):
        """Verify LIDAR is publishing."""
        collector = self.create_message_collector("/scan", LaserScan)
        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        assert len(messages) > 0, "No LIDAR messages"

        # Check data quality
        scan = messages[-1]
        assert len(scan.ranges) > 100, f"Expected >100 ranges, got {len(scan.ranges)}"

    def test_odometry_publishes(self):
        """Verify odometry is publishing."""
        collector = self.create_message_collector("/odom", Odometry)
        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert len(messages) > 0, "No odometry messages"
```

Run:

```bash
pytest test_turtlebot3_basic.py -v
```

## Automatic Simulation Control

Let sim_harness manage the simulation lifecycle:

```python
#!/usr/bin/env python3
"""TurtleBot3 test with automatic simulation control."""

import math
import pytest
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped

from sim_harness import SimulationTestFixture, ValidationResultCollector


@pytest.mark.integration
class TestTurtleBot3WithSim(SimulationTestFixture):
    """Tests that automatically start/stop simulation."""

    # Simulation configuration
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'
    LAUNCH_ARGS = {'use_sim_time': 'true'}
    ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}

    # Timing
    STARTUP_TIMEOUT = 60.0
    GAZEBO_STARTUP_DELAY = 10.0

    def test_sensors_operational(self):
        """Verify all sensors are publishing."""
        scan_col = self.create_message_collector("/scan", LaserScan, key="scan")
        odom_col = self.create_message_collector("/odom", Odometry, key="odom")

        self.spin_for_duration(5.0)

        # Check LIDAR
        scan_ok = scan_col.message_count() > 0
        self.validate_requirement(
            "REQ-SEN-001",
            "LIDAR publishes scan data",
            scan_ok,
            f"Received {scan_col.message_count()} messages",
            "Sensors"
        )

        # Check odometry
        odom_ok = odom_col.message_count() > 0
        self.validate_requirement(
            "REQ-SEN-002",
            "Odometry publishes pose data",
            odom_ok,
            f"Received {odom_col.message_count()} messages",
            "Sensors"
        )

        assert scan_ok and odom_ok, "Sensors not operational"

    def test_robot_responds_to_commands(self):
        """Verify robot moves when commanded."""
        # Collect odometry
        odom_col = self.create_message_collector("/odom", Odometry)
        self.spin_for_duration(1.0)

        initial_msgs = odom_col.get_messages()
        if not initial_msgs:
            pytest.skip("No initial odometry")

        initial_x = initial_msgs[-1].pose.pose.position.x

        # Send velocity commands
        cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)

        cmd = TwistStamped()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = 0.2  # 0.2 m/s forward

        import time
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop
        cmd.twist.linear.x = 0.0
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.5)

        # Check movement
        final_x = odom_col.get_messages()[-1].pose.pose.position.x
        distance = abs(final_x - initial_x)

        moved = distance > 0.1

        self.validate_requirement(
            "REQ-MOT-001",
            "Robot responds to velocity commands",
            moved,
            f"Moved {distance:.3f}m",
            "Motion"
        )

        self.node.destroy_publisher(cmd_pub)
        assert moved, f"Robot didn't move: {distance:.3f}m"


# Print validation summary
@pytest.fixture(scope="session", autouse=True)
def print_summary():
    yield
    ValidationResultCollector.instance().print_summary()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
```

Run:

```bash
pytest test_turtlebot3_with_sim.py -v -s
```

## Navigation Stack Testing

Test the full Nav2 stack:

```python
@pytest.mark.integration
@pytest.mark.slow
class TestTurtleBot3Navigation(SimulationTestFixture):
    """Navigation tests - requires Nav2 stack."""

    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'
    LAUNCH_ARGS = {'use_sim_time': 'true'}
    ENV_VARS = {'TURTLEBOT3_MODEL': 'waffle'}

    STARTUP_TIMEOUT = 120.0  # Nav2 takes longer
    GAZEBO_STARTUP_DELAY = 15.0

    def test_nav2_nodes_active(self):
        """Verify Nav2 lifecycle nodes are active."""
        from sim_harness import assert_lifecycle_node_active

        nodes = [
            "controller_server",
            "planner_server",
            "bt_navigator",
        ]

        for node_name in nodes:
            result = assert_lifecycle_node_active(
                self.node,
                node_name,
                timeout_sec=30.0
            )

            self.validate_requirement(
                f"REQ-NAV-{node_name}",
                f"{node_name} is active",
                result.success,
                result.details,
                "Navigation"
            )

    def test_costmap_available(self):
        """Verify costmap is being published."""
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        collector = self.create_message_collector(
            "/local_costmap/costmap",
            OccupancyGrid,
            qos_profile=qos
        )

        self.spin_for_duration(10.0)

        has_costmap = collector.message_count() > 0

        self.validate_requirement(
            "REQ-NAV-COSTMAP",
            "Local costmap is available",
            has_costmap,
            f"Received {collector.message_count()} costmap messages",
            "Navigation"
        )

        assert has_costmap, "No costmap messages"
```

## Running the Examples

```bash
# Run basic tests (requires simulation running)
pytest examples/test_turtlebot3_integration.py -v -k "Basic"

# Run with automatic simulation
pytest examples/test_turtlebot3_with_sim_control.py -v -s

# Run only integration tests
pytest -m integration -v

# Skip slow navigation tests
pytest -m "not slow" -v

# Export results to JSON
pytest examples/ -v && cat /tmp/turtlebot3_test_results.json
```

## Output Example

```
================================================================================
REQUIREMENT VALIDATION SUMMARY
================================================================================

Category: Sensors
--------------------------------------------------------------------------------
[PASS] REQ-SEN-001: LIDAR publishes scan data
       Details: Received 150 messages
[PASS] REQ-SEN-002: Odometry publishes pose data
       Details: Received 300 messages

Category: Motion
--------------------------------------------------------------------------------
[PASS] REQ-MOT-001: Robot responds to velocity commands
       Details: Moved 0.523m

================================================================================
SUMMARY: 3 passed, 0 failed, 0 skipped
================================================================================
```
