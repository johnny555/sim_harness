# Custom Robot Example

This example shows how to test your own robot with sim_harness.

## Project Structure

```
my_robot_ws/
├── src/
│   ├── my_robot_description/   # URDF, meshes
│   ├── my_robot_gazebo/        # Simulation launch files
│   └── my_robot_tests/         # Your tests
│       ├── test/
│       │   └── test_my_robot.py
│       └── conftest.py
```

## Create a Custom Fixture

Create a base fixture for your robot:

```python
# my_robot_tests/conftest.py
"""Test configuration for my_robot tests."""

import pytest
from sim_harness import SimulationTestFixture, ValidationResultCollector


class MyRobotTestFixture(SimulationTestFixture):
    """Base fixture for my_robot tests."""

    # Simulation configuration
    LAUNCH_PACKAGE = 'my_robot_gazebo'
    LAUNCH_FILE = 'simulation.launch.py'
    LAUNCH_ARGS = {
        'use_sim_time': 'true',
        'world': 'warehouse.sdf',
    }

    # Robot-specific topics
    SCAN_TOPIC = "/my_robot/scan"
    ODOM_TOPIC = "/my_robot/odom"
    CMD_VEL_TOPIC = "/my_robot/cmd_vel"
    IMU_TOPIC = "/my_robot/imu"

    # Timing
    STARTUP_TIMEOUT = 90.0
    GAZEBO_STARTUP_DELAY = 10.0


@pytest.fixture(scope="session", autouse=True)
def validation_report():
    """Generate validation report at end of session."""
    yield
    collector = ValidationResultCollector.instance()
    collector.print_summary()
    collector.export_to_json("/tmp/my_robot_validation.json")
```

## Write Your Tests

```python
# my_robot_tests/test/test_my_robot.py
"""Integration tests for my_robot."""

import math
import pytest
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# Import from conftest
import sys
sys.path.insert(0, '..')
from conftest import MyRobotTestFixture


class TestMyRobotSensors(MyRobotTestFixture):
    """Sensor tests for my_robot."""

    def test_lidar_publishes(self):
        """REQ-SEN-001: LIDAR publishes scan data."""
        collector = self.create_message_collector(
            self.SCAN_TOPIC,
            LaserScan
        )

        self.spin_for_duration(5.0)

        messages = collector.get_messages()
        passed = len(messages) > 0

        if passed:
            scan = messages[-1]
            details = f"Received {len(messages)} msgs, {len(scan.ranges)} ranges"
        else:
            details = f"No messages on {self.SCAN_TOPIC}"

        self.assert_requirement(
            "REQ-SEN-001",
            "LIDAR publishes scan data",
            passed,
            details,
            "Sensors"
        )

    def test_odometry_publishes(self):
        """REQ-SEN-002: Odometry publishes pose data."""
        collector = self.create_message_collector(
            self.ODOM_TOPIC,
            Odometry
        )

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        passed = len(messages) > 0

        self.assert_requirement(
            "REQ-SEN-002",
            "Odometry publishes pose data",
            passed,
            f"Received {len(messages)} messages",
            "Sensors"
        )

    def test_imu_publishes(self):
        """REQ-SEN-003: IMU publishes data."""
        collector = self.create_message_collector(
            self.IMU_TOPIC,
            Imu
        )

        self.spin_for_duration(3.0)

        messages = collector.get_messages()
        passed = len(messages) > 0

        self.assert_requirement(
            "REQ-SEN-003",
            "IMU publishes orientation data",
            passed,
            f"Received {len(messages)} messages",
            "Sensors"
        )

    def test_sensor_data_quality(self):
        """REQ-SEN-004: Sensor data contains no NaN values."""
        scan_col = self.create_message_collector(self.SCAN_TOPIC, LaserScan)
        odom_col = self.create_message_collector(self.ODOM_TOPIC, Odometry)

        self.spin_for_duration(5.0)

        issues = []

        # Check odometry
        for odom in odom_col.get_messages():
            if math.isnan(odom.pose.pose.position.x):
                issues.append("Odometry has NaN")
                break

        # Check LIDAR (allow some inf for out-of-range)
        for scan in scan_col.get_messages():
            nan_count = sum(1 for r in scan.ranges if math.isnan(r))
            if nan_count > len(scan.ranges) * 0.1:  # >10% NaN
                issues.append(f"LIDAR has {nan_count} NaN values")
                break

        passed = len(issues) == 0
        details = "All data valid" if passed else "; ".join(issues)

        self.assert_requirement(
            "REQ-SEN-004",
            "Sensor data is valid (no NaN)",
            passed,
            details,
            "Sensors"
        )


class TestMyRobotMotion(MyRobotTestFixture):
    """Motion tests for my_robot."""

    def test_robot_moves_forward(self):
        """REQ-MOT-001: Robot moves when commanded."""
        odom_col = self.create_message_collector(self.ODOM_TOPIC, Odometry)
        self.spin_for_duration(1.0)

        if odom_col.message_count() == 0:
            pytest.skip("No odometry available")

        initial_x = odom_col.get_latest().pose.pose.position.x

        # Send velocity
        cmd_pub = self.node.create_publisher(Twist, self.CMD_VEL_TOPIC, 10)
        cmd = Twist()
        cmd.linear.x = 0.5

        import time
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        # Stop
        cmd.linear.x = 0.0
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.5)

        final_x = odom_col.get_latest().pose.pose.position.x
        distance = abs(final_x - initial_x)
        passed = distance > 0.5

        self.assert_requirement(
            "REQ-MOT-001",
            "Robot responds to velocity commands",
            passed,
            f"Moved {distance:.2f}m",
            "Motion"
        )

        self.node.destroy_publisher(cmd_pub)

    def test_robot_turns(self):
        """REQ-MOT-002: Robot can turn in place."""
        odom_col = self.create_message_collector(self.ODOM_TOPIC, Odometry)
        self.spin_for_duration(1.0)

        if odom_col.message_count() == 0:
            pytest.skip("No odometry available")

        def get_yaw(odom):
            q = odom.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny, cosy)

        initial_yaw = get_yaw(odom_col.get_latest())

        # Send rotation
        cmd_pub = self.node.create_publisher(Twist, self.CMD_VEL_TOPIC, 10)
        cmd = Twist()
        cmd.angular.z = 0.5  # rad/s

        import time
        start = time.monotonic()
        while time.monotonic() - start < 3.0:
            cmd_pub.publish(cmd)
            self.spin_for_duration(0.1)

        cmd.angular.z = 0.0
        cmd_pub.publish(cmd)
        self.spin_for_duration(0.5)

        final_yaw = get_yaw(odom_col.get_latest())
        rotation = abs(final_yaw - initial_yaw)
        passed = rotation > 0.5  # At least ~30 degrees

        self.assert_requirement(
            "REQ-MOT-002",
            "Robot can rotate in place",
            passed,
            f"Rotated {math.degrees(rotation):.1f} degrees",
            "Motion"
        )

        self.node.destroy_publisher(cmd_pub)


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
```

## Running Your Tests

```bash
# Run all tests
cd my_robot_tests
pytest test/ -v -s

# Run only sensor tests
pytest test/ -v -k "Sensors"

# Run without simulation (use existing)
pytest test/ -v --use-existing-sim

# Generate JUnit XML for CI
pytest test/ -v --junitxml=results.xml
```

## Adding to CI/CD

```yaml
# .github/workflows/test.yml
name: Robot Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    container:
      image: ros:humble

    steps:
      - uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          apt-get update
          apt-get install -y python3-pip
          pip3 install sim-harness pytest

      - name: Build workspace
        run: |
          source /opt/ros/humble/setup.bash
          colcon build

      - name: Run tests
        run: |
          source install/setup.bash
          pytest src/my_robot_tests/test/ -v --junitxml=results.xml

      - name: Upload results
        uses: actions/upload-artifact@v3
        with:
          name: test-results
          path: |
            results.xml
            /tmp/my_robot_validation.json
```

## Tips for Custom Robots

!!! tip "Use namespaced topics"
    If your robot uses namespaced topics (`/my_robot/scan`), define them in your fixture class for consistency.

!!! tip "Adjust timeouts"
    Complex simulations may need longer startup times. Increase `STARTUP_TIMEOUT` and `GAZEBO_STARTUP_DELAY`.

!!! tip "Create helper methods"
    Add common operations to your fixture:
    ```python
    def move_forward(self, distance: float, velocity: float = 0.5):
        """Helper to move robot forward."""
        # Implementation here
    ```

!!! warning "Handle simulation quirks"
    Some simulations need warm-up time. Add initial delays if tests are flaky:
    ```python
    def setup_method(self):
        super().setup_method()
        self.spin_for_duration(2.0)  # Warm-up
    ```
