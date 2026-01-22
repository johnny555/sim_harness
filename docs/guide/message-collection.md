# Message Collection

Message collectors simplify subscribing to topics and buffering messages for assertions.

## Basic Usage

```python
from sensor_msgs.msg import LaserScan
from sim_harness import SimTestFixture

class TestSensors(SimTestFixture):
    def test_lidar(self):
        # Create a collector
        collector = self.create_message_collector("/scan", LaserScan)

        # Wait for messages
        self.spin_for_duration(3.0)

        # Get all collected messages
        messages = collector.get_messages()
        assert len(messages) > 0
```

## Collector Options

### Key Parameter

Use `key` to retrieve collectors later:

```python
def test_multiple_topics(self):
    self.create_message_collector("/scan", LaserScan, key="scan")
    self.create_message_collector("/odom", Odometry, key="odom")

    self.spin_for_duration(3.0)

    # Retrieve by key
    scan_msgs = self.get_collector("scan").get_messages()
    odom_msgs = self.get_collector("odom").get_messages()
```

### Buffer Size

Limit memory usage with `max_messages`:

```python
# Only keep the last 100 messages
collector = self.create_message_collector(
    "/scan",
    LaserScan,
    max_messages=100
)
```

### Custom QoS

Specify QoS settings for the subscription:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

collector = self.create_message_collector(
    "/map",
    OccupancyGrid,
    qos_profile=qos
)
```

## Accessing Messages

### Get All Messages

```python
messages = collector.get_messages()
for msg in messages:
    print(msg.header.stamp)
```

### Get Latest Message

```python
latest = collector.get_latest()
if latest:
    print(f"Latest scan has {len(latest.ranges)} ranges")
```

### Get Message Count

```python
count = collector.message_count()
assert count >= 10, "Expected at least 10 messages"
```

### Wait for Messages

```python
# Wait until at least 5 messages received (or timeout)
success = collector.wait_for_messages(count=5, timeout=10.0)
assert success, "Didn't receive enough messages"
```

### Clear Messages

```python
# Clear the buffer
collector.clear()
assert collector.message_count() == 0
```

## Filtering Messages

### Filter by Callback

```python
def test_filter_valid_scans(self):
    collector = self.create_message_collector("/scan", LaserScan)
    self.spin_for_duration(5.0)

    # Filter to only valid scans (no NaN values)
    valid_scans = [
        msg for msg in collector.get_messages()
        if not any(math.isnan(r) for r in msg.ranges)
    ]

    assert len(valid_scans) > 0
```

### Filter by Time

```python
def test_recent_messages(self):
    collector = self.create_message_collector("/odom", Odometry)
    self.spin_for_duration(5.0)

    # Get messages from the last 2 seconds
    now = self.node.get_clock().now()
    recent = [
        msg for msg in collector.get_messages()
        if (now - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds < 2e9
    ]
```

## Multiple Collectors

### Parallel Collection

Collect from multiple topics simultaneously:

```python
def test_sensor_fusion(self):
    scan_col = self.create_message_collector("/scan", LaserScan, key="scan")
    odom_col = self.create_message_collector("/odom", Odometry, key="odom")
    imu_col = self.create_message_collector("/imu", Imu, key="imu")

    # All collectors receive messages during spin
    self.spin_for_duration(3.0)

    # Check all sensors
    assert scan_col.message_count() > 0, "No LIDAR"
    assert odom_col.message_count() > 0, "No odometry"
    assert imu_col.message_count() > 0, "No IMU"
```

### Synchronized Collection

Check message timing across topics:

```python
def test_synchronized_data(self):
    scan_col = self.create_message_collector("/scan", LaserScan)
    odom_col = self.create_message_collector("/odom", Odometry)

    self.spin_for_duration(3.0)

    # Check timestamps are close
    scan_time = scan_col.get_latest().header.stamp
    odom_time = odom_col.get_latest().header.stamp

    time_diff = abs(
        rclpy.time.Time.from_msg(scan_time).nanoseconds -
        rclpy.time.Time.from_msg(odom_time).nanoseconds
    )

    # Should be within 100ms
    assert time_diff < 1e8, "Sensor timestamps too far apart"
```

## Checking Message Rates

```python
def test_lidar_rate(self):
    """Verify LIDAR publishes at expected rate."""
    collector = self.create_message_collector("/scan", LaserScan)

    # Collect for exactly 5 seconds
    self.spin_for_duration(5.0)

    count = collector.message_count()
    rate = count / 5.0

    # TurtleBot3 LIDAR is 5Hz
    assert 4.0 <= rate <= 6.0, f"LIDAR rate {rate}Hz outside expected 5Hz"
```

## Best Practices

!!! tip "Clear collectors between phases"
    If your test has multiple phases, clear collectors to get fresh data:
    ```python
    collector.clear()
    self.spin_for_duration(2.0)  # Fresh messages only
    ```

!!! tip "Use appropriate buffer sizes"
    For high-frequency topics, limit buffer size to avoid memory issues:
    ```python
    # 1000Hz IMU - only keep last second
    collector = self.create_message_collector("/imu", Imu, max_messages=1000)
    ```

!!! warning "Don't forget to spin"
    Collectors only receive messages when the node spins:
    ```python
    collector = self.create_message_collector("/scan", LaserScan)
    # BAD: No messages yet!
    assert collector.message_count() > 0  # Will fail!

    # GOOD: Spin first
    self.spin_for_duration(1.0)
    assert collector.message_count() > 0  # Now it works
    ```
