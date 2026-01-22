# Writing Tests

This guide covers the patterns and best practices for writing sim_harness tests.

## Test Structure

A typical test file follows this structure:

```python
#!/usr/bin/env python3
"""Description of what this test file covers."""

import pytest
from sensor_msgs.msg import LaserScan

from sim_harness import SimTestFixture


class TestSensorIntegration(SimTestFixture):
    """Test class docstring - describes the test suite."""

    # Class-level configuration
    SCAN_TOPIC = "/scan"

    def test_something(self):
        """Test method docstring - describes this specific test."""
        # Arrange - set up test conditions
        collector = self.create_message_collector(self.SCAN_TOPIC, LaserScan)

        # Act - perform the action
        self.spin_for_duration(3.0)

        # Assert - verify the results
        messages = collector.get_messages()
        assert len(messages) > 0
```

## Test Categories

### Basic Tests

Tests that verify fundamental functionality without complex setup:

```python
def test_basic_topic_publishes(self):
    """Verify a topic is publishing - should always pass if sim running."""
    collector = self.create_message_collector("/scan", LaserScan)
    self.spin_for_duration(3.0)
    assert len(collector.get_messages()) > 0
```

### Integration Tests

Tests that verify multiple components working together:

```python
@pytest.mark.integration
def test_navigation_stack(self):
    """Verify navigation stack is operational."""
    # Check multiple nav2 nodes are active
    nodes = ["controller_server", "planner_server", "bt_navigator"]
    for node in nodes:
        result = assert_lifecycle_node_active(self.node, node)
        assert result.success, f"{node} not active"
```

### Motion Tests

Tests that involve robot movement:

```python
def test_robot_moves_forward(self):
    """Verify robot responds to velocity commands."""
    # Get initial position
    odom = self.create_message_collector("/odom", Odometry)
    self.spin_for_duration(1.0)
    initial_x = odom.get_messages()[-1].pose.pose.position.x

    # Send velocity command
    cmd_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
    cmd = Twist()
    cmd.linear.x = 0.5

    for _ in range(30):  # 3 seconds at 10Hz
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.1)

    # Stop and check
    cmd.linear.x = 0.0
    cmd_pub.publish(cmd)
    self.spin_for_duration(0.5)

    final_x = odom.get_messages()[-1].pose.pose.position.x
    distance = abs(final_x - initial_x)

    assert distance > 0.5, f"Robot only moved {distance}m"
```

## Using Markers

pytest markers help organize and filter tests:

```python
import pytest

@pytest.mark.slow
def test_long_running_navigation(self):
    """Test that takes a long time."""
    pass

@pytest.mark.integration
def test_full_stack(self):
    """Requires full navigation stack."""
    pass

@pytest.mark.requires_sim
def test_needs_gazebo(self):
    """Only runs if simulation is available."""
    pass
```

Run specific markers:

```bash
# Run only integration tests
pytest -m integration

# Skip slow tests
pytest -m "not slow"

# Run basic tests only
pytest -k "basic"
```

## Test Isolation

Each test should be independent:

```python
class TestIsolation(SimTestFixture):

    def test_first(self):
        """This test should not affect test_second."""
        collector = self.create_message_collector("/scan", LaserScan)
        self.spin_for_duration(2.0)
        # Collector is automatically cleaned up

    def test_second(self):
        """This test starts fresh."""
        collector = self.create_message_collector("/scan", LaserScan)
        self.spin_for_duration(2.0)
        # Gets its own collector, not affected by test_first
```

## Error Handling

Handle cases where tests can't run:

```python
def test_with_skip(self):
    """Skip if preconditions not met."""
    collector = self.create_message_collector("/odom", Odometry)
    self.spin_for_duration(2.0)

    messages = collector.get_messages()
    if len(messages) < 2:
        pytest.skip("Not enough odometry data to test")

    # Continue with test...
```

## Logging

Use the built-in logger:

```python
def test_with_logging(self):
    """Test that logs useful information."""
    self.get_logger().info("Starting sensor test")

    collector = self.create_message_collector("/scan", LaserScan)
    self.spin_for_duration(3.0)

    messages = collector.get_messages()
    self.get_logger().info(f"Received {len(messages)} messages")

    if messages:
        self.get_logger().debug(f"Last scan had {len(messages[-1].ranges)} ranges")
```

## Best Practices

!!! tip "Keep tests focused"
    Each test should verify one thing. If you're testing LIDAR and odometry, write two separate tests.

!!! tip "Use descriptive names"
    `test_lidar_publishes_at_10hz` is better than `test_lidar`.

!!! tip "Set appropriate timeouts"
    Don't wait longer than necessary. 3-5 seconds is usually enough for topic checks.

!!! tip "Handle flaky tests"
    If a test is timing-dependent, consider using retries or longer timeouts rather than marking it as expected to fail.

!!! warning "Avoid test interdependence"
    Tests should not depend on other tests running first. Each test should set up its own conditions.
