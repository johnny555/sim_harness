# Your First Test

Let's write a simple test for a ROS 2 robot.

## Create a Test File

Create a new file `test_my_robot.py`:

```python
#!/usr/bin/env python3
"""My first sim_harness test."""

import pytest
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from sim_harness import SimTestFixture


class TestMyRobot(SimTestFixture):
    """Test suite for my robot."""

    def test_lidar_publishes(self):
        """Verify LIDAR sensor is publishing data."""
        # Create a message collector for the scan topic
        collector = self.create_message_collector(
            "/scan",
            LaserScan,
            key="scan"
        )

        # Wait for messages (spins the ROS node)
        self.spin_for_duration(5.0)

        # Get collected messages
        messages = collector.get_messages()

        # Assert we received data
        assert len(messages) > 0, "No LIDAR messages received"

        # Check data quality
        scan = messages[-1]
        assert len(scan.ranges) > 0, "LIDAR scan has no ranges"

    def test_odometry_publishes(self):
        """Verify odometry is being published."""
        collector = self.create_message_collector(
            "/odom",
            Odometry,
            key="odom"
        )

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        assert len(messages) > 0, "No odometry messages received"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

## Run the Test

Start your robot simulation, then run:

```bash
pytest test_my_robot.py -v
```

## Understanding the Code

### Test Fixture

```python
class TestMyRobot(SimTestFixture):
```

`SimTestFixture` is the base class that:

- Creates a ROS 2 node for your test
- Handles setup and teardown
- Provides helper methods

### Message Collection

```python
collector = self.create_message_collector("/scan", LaserScan, key="scan")
```

This creates a subscriber that buffers messages. The `key` parameter lets you retrieve the collector later if needed.

### Spinning

```python
self.spin_for_duration(5.0)
```

This spins the ROS node for 5 seconds, allowing:

- Subscribers to receive messages
- Publishers to send data
- Callbacks to execute

### Assertions

```python
assert len(messages) > 0, "No LIDAR messages received"
```

Use standard pytest assertions. The message after the comma is shown on failure.

## Adding Requirement Tracing

Map your tests to requirements:

```python
def test_lidar_publishes(self):
    collector = self.create_message_collector("/scan", LaserScan)
    self.spin_for_duration(5.0)
    messages = collector.get_messages()

    passed = len(messages) > 0

    # Record requirement validation
    self.assert_requirement(
        req_id="REQ-SEN-001",
        description="LIDAR sensor publishes scan data",
        passed=passed,
        details=f"Received {len(messages)} messages",
        category="Sensors"
    )

    assert passed, "LIDAR test failed"
```

## Next Steps

- [Test Fixtures](../guide/fixtures.md) - Learn about fixture types
- [Message Collection](../guide/message-collection.md) - Advanced collection patterns
- [Assertions](../guide/assertions.md) - Built-in assertion functions
