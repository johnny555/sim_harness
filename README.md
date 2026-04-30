# sim_harness

A reusable ROS 2 test harness for simulation-based integration testing.

`sim_harness` is a Python library and pytest plugin: pytest fixtures that
spin up a `rclpy` node + `MessageCollector` for each test, a check-function
library (`checks`, `nav2`, `perception`, `navigation_quality`), launch
helpers, and a `@pytest.mark.requirement` marker that the plugin exports to
a Jama-importable spreadsheet via `--jama-xlsx`.

It is deliberately project- and robot-agnostic: topic names, vehicle types,
BT node names, and world content are supplied by the consumer.

## Quick start

`sim_harness` registers two pytest plugins via `pytest11` entry points so
they auto-load:

- `sim_harness_ros` — provides `ros_node` + `message_collector_factory` (and
  a session-scoped `ros_context` that handles `rclpy.init()` /
  `rclpy.shutdown()`).
- `sim_harness_jama` — adds `@pytest.mark.requirement(...)` and the
  `--jama-xlsx PATH` exporter.

A test module bringing up a Gazebo launch with `launch_pytest`:

```python
import launch
import launch_pytest
import pytest
from sensor_msgs.msg import LaserScan
from sim_harness.checks import check_lidar_valid

@launch_pytest.fixture(scope='module')
def my_sim_launch():
    return launch.LaunchDescription([
        # ... your launch actions ...
        launch_pytest.actions.ReadyToTest(),
    ])

@pytest.mark.launch(fixture=my_sim_launch)
@pytest.mark.requirement("REQ-SEN-001", "LIDAR publishes valid scans",
                         category="Sensors")
def test_lidar(my_sim_launch, ros_node, message_collector_factory):
    scans = message_collector_factory(LaserScan, '/scan')
    ros_node.spin_until(lambda: scans.count() > 0, timeout_sec=10.0)
    result = check_lidar_valid(ros_node.node, '/scan')
    assert result.ok, result.details
```

Tests are plain pytest functions: declare what fixtures they need, no
`unittest.TestCase` ceremony. The legacy `SimTestFixture` class
(`sim_harness.fixture`) still exists for the older `class TestX(SimTestFixture)`
style but new code should prefer the fixture-based pattern shown above.

## Running tests

The DCE workspace ships a thin wrapper (`run_test.sh`) and a `tasks.mk`
of common targets so tests can be run with one command:

```bash
# from the workspace root (where install/ lives)
./run_test.sh                        # full pytest suite
./run_test.sh -m marionette          # marker filter
./run_test.sh -m "not gazebo"        # skip Gazebo-backed tests
./run_test.sh --jama-xlsx /tmp/r.xlsx  # with Jama xlsx export

# convenience targets
make -f tasks.mk test                # full suite
make -f tasks.mk test-fast           # marker filter "not gazebo"
make -f tasks.mk test-marionette     # marionette-tagged only
make -f tasks.mk jama                # full suite + xlsx
make -f tasks.mk help                # list all targets
```

`run_test.sh` sets `SMOKE_HEADLESS=true`, `ROS_DOMAIN_ID=142` (override
by exporting it), `VGL_DISPLAY=egl`, and sources `install/setup.bash`
relative to its own location. The Jama export comes from this package's
auto-loaded pytest plugin (see "Tagging tests with Jama requirement IDs"
below).

Markers are declared in `src/simulator/pytest.ini`:

| Marker          | Meaning                                                  |
|-----------------|----------------------------------------------------------|
| `gazebo`        | requires a running Gazebo simulator (slow)               |
| `marionette`    | covers the Marionette UDP protocol or its node           |
| `nav2`          | exercises Nav2 lifecycle / planner / controller          |
| `validation`    | validates a numbered system requirement                  |
| `requires_sim`  | needs an externally-running simulator                    |

Compose them with boolean expressions:

```bash
./run_test.sh -m "marionette and not gazebo"   # protocol unit tests only
./run_test.sh -m "validation and not gazebo"   # config-only validation
```

## Layers

| Layer | Module                       | Purpose                                              |
|-------|------------------------------|------------------------------------------------------|
| 0     | `sim_harness.pytest_ros_fixtures` | Pytest fixtures: `ros_context`, `ros_node`, `message_collector_factory` |
| 0     | `sim_harness.fixture`        | `SimTestFixture` — legacy class-based base          |
| 0     | `sim_harness.spin`           | Spin helpers (`spin_for_duration`, `spin_until_*`)  |
| 0     | `sim_harness.collector`      | `MessageCollector` for topic sampling                |
| 1     | `sim_harness.checks`         | Generic sensor/service/motion checks                 |
| 1     | `sim_harness.nav2`           | Nav2 lifecycle + action checks                       |
| 1     | `sim_harness.perception`     | Detection / perception checks                        |
| 1     | `sim_harness.navigation_quality` | Motion quality tracker + pluggable monitors     |
| 1     | `sim_harness.pytest_jama_plugin` | Pytest plugin: `@pytest.mark.requirement` + `--jama-xlsx` |
| 2     | `sim_harness.simulator`      | `SimulatorInterface`, `GazeboBackend`, lifecycle    |
| 2     | `sim_harness.launch_utils`   | `chain_on_exit`, Gazebo env setup                   |

Lower layers never import higher ones. Consumers can use any layer without
the others.

## Extension points

`sim_harness` is designed to be configured, not forked. Use these hooks to
add project-specific behaviour without modifying the library.

### Tagging tests with Jama requirement IDs

The `pytest_jama_plugin` auto-loads via the `pytest11` entry point. Tag
tests with `@pytest.mark.requirement(...)` and the plugin will write a
Jama-importable spreadsheet when `--jama-xlsx PATH` is supplied:

```python
import pytest

@pytest.mark.requirement("REQ-SEN-001", "LIDAR publishes valid data",
                         category="Sensors")
def test_lidar(node):
    ...
    assert ...

# pytest --jama-xlsx /tmp/results.xlsx --jama-project DCE
```

The xlsx has two sheets: detailed `Test Results` (per-test rows including
the Requirement ID, status, duration, and failure message) and `Summary`
(pass/fail counts, total duration, project key). When `--jama-xlsx` is
omitted, the plugin is a no-op — tests still run, just no spreadsheet.

### Pluggable navigation-quality monitors

`NavigationQualityTracker` accumulates motion metrics; the monitors that
feed it are separate factories so you supply your own topic names and
behaviour-tree node→phase mapping:

```python
from sim_harness.navigation_quality import (
    NavigationQualityTracker,
    monitor_phases_from_bt_log,
    monitor_collisions,
)

PHASE_NODES = {'ForwardPlan': 'Forward', 'ReversePath': 'Reverse'}
COLLISION_SOURCES = {'truck': '/truck_contacts',
                     'trailer': '/trailer_contacts'}

tracker = NavigationQualityTracker(goal_x=10, goal_y=0, goal_yaw=0)
monitor_phases_from_bt_log(node, f'/{vehicle_id}/behavior_tree_log',
                            PHASE_NODES, phases_seen=set())
monitor_collisions(node, COLLISION_SOURCES, tracker=tracker)
```

The mapper accepts a `dict` *or* a callable `(bt_node_name) -> phase_label
or None`. Collision sources accept a dict, a list of topic names, or a
list of `(label, topic)` pairs.

### Predicate-based launch gating

Launch files often use `TimerAction(period=N)` to wait for Gazebo, lifecycle
nodes, or service registration. Fixed delays are brittle across machines —
a headless CI box needs 10 s where a laptop loading Sellafield needs 120 s.
`WaitForCondition` polls a real predicate and advances as soon as it's
true:

```python
from launch.actions import OpaqueFunction
from sim_harness.launch_utils import (
    WaitForCondition, topic_publishing, lifecycle_active,
)

# Gate Nav2 startup on /clock actually publishing (not a fixed 10–120 s timer).
wait_for_clock = WaitForCondition(
    condition=topic_publishing(
        '/clock', 'rosgraph_msgs/msg/Clock', min_rate_hz=5.0),
    actions=[OpaqueFunction(function=launch_nav2_setup)],
    timeout=300.0,
    description='Wait for /clock',
)

# Gate lifecycle manager on bt_navigator reaching 'active'.
wait_for_bt = WaitForCondition(
    condition=lifecycle_active('/robot/bt_navigator'),
    actions=[lifecycle_manager_navigation],
    timeout=60.0,
    description='Wait for bt_navigator active',
)
```

Factories returning ready-to-use condition callables:

- `topic_publishing(topic, msg_type, min_rate_hz, sample_window_sec)`
- `service_available(service_name)`
- `lifecycle_active(node_name)`

Any `Callable[[], bool]` works — roll your own if none of the factories fit.
Timeouts fail closed by default and shut the launch down. Use
`on_timeout='proceed'` only for intentional best-effort flows where running
the follow-up actions without the predicate is acceptable.

### Custom simulator backends

`SimulatorInterface` is abstract. `GazeboBackend` and `NullBackend` ship
with the library; add a new backend (Isaac Sim, MuJoCo, etc.) by
subclassing `SimulatorInterface` and registering it with
`SimulationManager`.

## Non-goals

`sim_harness` deliberately does **not**:

- Embed vehicle-, world-, or topic-name assumptions. Pass those in from
  the consumer (robot namespace, contact sensor topics, BT node names).
- Ship launch files for any specific robot. Consumers provide their own
  via `LAUNCH_PACKAGE` / `LAUNCH_FILE`.
- Depend on any project-internal package. All dependencies are listed in
  `package.xml`.
- Manage workspace configuration, build parameters, or CI plumbing.

If you find yourself tempted to add project-specific logic to this
library, prefer exposing a pluggable hook (like the phase mapper or
collision sources) so the project can configure the library from outside.

## Layout

```
sim_harness/
├── sim_harness/                # Python package
│   ├── pytest_ros_fixtures.py  # ros_node + message_collector_factory (Layer 0)
│   ├── fixture.py              # SimTestFixture (legacy class-based, Layer 0)
│   ├── spin.py                 # Spin helpers (Layer 0)
│   ├── collector.py            # MessageCollector (Layer 0)
│   ├── checks/                 # Generic checks (Layer 1, split by concern)
│   ├── nav2.py                 # Nav2 checks (Layer 1)
│   ├── perception.py           # Detection checks (Layer 1)
│   ├── navigation_quality.py   # Motion tracker + monitors (Layer 1)
│   ├── pytest_jama_plugin.py   # @pytest.mark.requirement + --jama-xlsx
│   ├── simulator/              # Simulator lifecycle (Layer 2)
│   └── launch_utils.py         # launch helpers (Layer 2)
├── launch/                     # Reusable launch fragments
├── test/                       # Unit tests for the harness itself
└── examples/                   # Reference integrations (e.g. turtlebot3)
```

## License

Apache-2.0. See `package.xml` for dependency declarations.
