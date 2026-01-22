# sim_harness

A ROS 2 simulation testing framework for robotics applications.

<div class="grid cards" markdown>

-   :material-clock-fast:{ .lg .middle } __Quick to Set Up__

    ---

    Install with pip and start writing tests in minutes.

    [:octicons-arrow-right-24: Getting Started](getting-started/installation.md)

-   :material-test-tube:{ .lg .middle } __pytest Integration__

    ---

    Uses familiar pytest patterns. No new test runner to learn.

    [:octicons-arrow-right-24: Writing Tests](guide/writing-tests.md)

-   :material-robot:{ .lg .middle } __Simulation Control__

    ---

    Automatically start and stop Gazebo simulations for your tests.

    [:octicons-arrow-right-24: Simulation Control](guide/simulation-control.md)

-   :material-check-all:{ .lg .middle } __Requirement Tracing__

    ---

    Map tests to requirements and generate compliance reports.

    [:octicons-arrow-right-24: Requirements](guide/requirements.md)

</div>

## What is sim_harness?

**sim_harness** is a testing framework designed for ROS 2 robotics applications that run in simulation. It provides:

- **Test Fixtures** - Base classes that handle ROS 2 node lifecycle
- **Message Collectors** - Easy subscription and message buffering
- **Simulation Control** - Automatic Gazebo lifecycle management
- **Assertion Primitives** - Pre-built checks for sensors, navigation, and more
- **Requirement Validation** - Map tests to requirements for traceability

## Quick Example

```python
from sim_harness import SimTestFixture
from sensor_msgs.msg import LaserScan

class TestMyRobot(SimTestFixture):
    def test_lidar_publishes(self):
        # Collect messages from a topic
        collector = self.create_message_collector("/scan", LaserScan)

        # Wait for messages
        self.spin_for_duration(3.0)

        # Assert we received data
        messages = collector.get_messages()
        assert len(messages) > 0, "No LIDAR messages received"
        assert len(messages[-1].ranges) > 100, "LIDAR should have > 100 ranges"
```

Run it with pytest:

```bash
pytest test_my_robot.py -v
```

## Features

### Simple Test Writing

Write tests using familiar pytest patterns. No complex setup required.

### Automatic Simulation Lifecycle

```python
class TestWithSim(SimulationTestFixture):
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'

    def test_something(self):
        # Simulation is already running!
        pass
```

### Built-in Assertions

```python
from sim_harness import assert_topic_published, assert_vehicle_moved

# Check a topic is publishing
result = assert_topic_published(node, "/scan", LaserScan, timeout=5.0)
assert result.success

# Check robot moved
result = assert_vehicle_moved(node, "robot_01", min_distance=1.0)
assert result.success
```

### Requirement Tracing

```python
def test_lidar_operational(self):
    # ... test code ...

    self.assert_requirement(
        "REQ-SEN-001",
        "LIDAR sensor publishes valid data",
        passed=True,
        details="Received 100 messages",
        category="Sensors"
    )
```

## Installation

```bash
pip install sim-harness
```

Or from source:

```bash
cd ~/ros2_ws/src
git clone https://github.com/johnny555/sim_harness.git
cd ..
colcon build --packages-select sim_harness
```

## License

sim_harness is released under the Apache 2.0 License.
