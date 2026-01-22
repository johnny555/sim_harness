# Test Fixtures

sim_harness provides test fixtures that handle ROS 2 node lifecycle and common setup tasks.

## Available Fixtures

### SimTestFixture

The basic fixture for tests where the simulation is already running:

```python
from sim_harness import SimTestFixture

class TestMyRobot(SimTestFixture):
    def test_something(self):
        # self.node is available - a fully initialized ROS 2 node
        pass
```

**Use when:**

- Simulation is started manually or externally
- Running quick tests during development
- Testing against a real robot

### SimulationTestFixture

Automatically starts and stops the simulation:

```python
from sim_harness import SimulationTestFixture

class TestWithSim(SimulationTestFixture):
    # Required: specify the launch file
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'

    def test_something(self):
        # Simulation is already running!
        pass
```

**Use when:**

- Running CI/CD tests
- Tests need a clean simulation state
- Automated test suites

## SimTestFixture Details

### Lifecycle

```python
class TestExample(SimTestFixture):

    @classmethod
    def setup_class(cls):
        """Called once before any tests in this class."""
        super().setup_class()
        # Add custom class-level setup

    def setup_method(self):
        """Called before each test method."""
        super().setup_method()
        # Add custom per-test setup

    def teardown_method(self):
        """Called after each test method."""
        # Add custom per-test cleanup
        super().teardown_method()

    @classmethod
    def teardown_class(cls):
        """Called once after all tests in this class."""
        # Add custom class-level cleanup
        super().teardown_class()
```

### Available Properties and Methods

| Property/Method | Description |
|-----------------|-------------|
| `self.node` | The ROS 2 node instance |
| `self.get_logger()` | Get the node's logger |
| `self.spin_for_duration(seconds)` | Spin the node for a duration |
| `self.create_message_collector(topic, msg_type)` | Create a message collector |
| `self.assert_requirement(...)` | Record a requirement validation |
| `self.validate_requirement(...)` | Record without asserting |

## SimulationTestFixture Details

### Configuration Options

```python
class TestWithSim(SimulationTestFixture):
    # Required
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'

    # Optional - launch arguments
    LAUNCH_ARGS = {
        'use_sim_time': 'true',
        'world': 'empty_world',
    }

    # Optional - environment variables
    ENV_VARS = {
        'TURTLEBOT3_MODEL': 'waffle',
    }

    # Optional - timing
    STARTUP_TIMEOUT = 60.0      # Max time to wait for sim (default: 60)
    GAZEBO_STARTUP_DELAY = 5.0  # Extra delay after Gazebo starts (default: 5)

    # Optional - behavior
    REQUIRE_SIM = True          # Fail if sim can't start (default: True)
    USE_EXISTING_SIM = False    # Use already-running sim (default: False)
```

### Using Existing Simulation

For faster iteration during development:

```python
class TestQuickIteration(SimulationTestFixture):
    USE_EXISTING_SIM = True   # Don't start/stop sim
    REQUIRE_SIM = True        # Skip if sim not running

    def test_something(self):
        # Uses the already-running simulation
        pass
```

### Simulation Reuse

The simulation manager reuses simulations when possible:

```python
class TestFirst(SimulationTestFixture):
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'

    def test_a(self):
        pass

class TestSecond(SimulationTestFixture):
    # Same launch config - simulation is reused!
    LAUNCH_PACKAGE = 'turtlebot3_gazebo'
    LAUNCH_FILE = 'turtlebot3_world.launch.py'

    def test_b(self):
        pass
```

## Custom Fixtures

Create your own fixture by extending the base:

```python
from sim_harness import SimTestFixture

class MyRobotTestFixture(SimTestFixture):
    """Custom fixture for my robot with common setup."""

    # Default topics for my robot
    SCAN_TOPIC = "/my_robot/scan"
    ODOM_TOPIC = "/my_robot/odom"
    CMD_VEL_TOPIC = "/my_robot/cmd_vel"

    def setup_method(self):
        super().setup_method()
        # Always create these collectors
        self.scan_collector = self.create_message_collector(
            self.SCAN_TOPIC, LaserScan
        )
        self.odom_collector = self.create_message_collector(
            self.ODOM_TOPIC, Odometry
        )

    def send_velocity(self, linear: float, angular: float, duration: float):
        """Helper to send velocity commands."""
        pub = self.node.create_publisher(Twist, self.CMD_VEL_TOPIC, 10)
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular

        import time
        start = time.monotonic()
        while time.monotonic() - start < duration:
            pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        pub.publish(cmd)
        self.node.destroy_publisher(pub)


# Use your custom fixture
class TestMyRobot(MyRobotTestFixture):
    def test_movement(self):
        initial = self.odom_collector.get_messages()[-1].pose.pose.position.x
        self.send_velocity(0.5, 0.0, 2.0)
        final = self.odom_collector.get_messages()[-1].pose.pose.position.x
        assert abs(final - initial) > 0.5
```

## Pytest Fixtures

You can also use standard pytest fixtures:

```python
import pytest
from sim_harness import SimTestFixture

@pytest.fixture
def robot_at_origin(request):
    """Fixture that ensures robot starts at origin."""
    # Setup: teleport robot to origin
    # ... teleport logic ...
    yield
    # Teardown: nothing needed

class TestNavigation(SimTestFixture):
    def test_navigate_to_goal(self, robot_at_origin):
        # Robot is guaranteed to be at origin
        pass
```
