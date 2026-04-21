# sim_harness

A reusable ROS 2 test harness for simulation-based integration testing.

`sim_harness` provides a pytest-friendly fixture, check functions, requirement
tracking, and a simulator-lifecycle abstraction. It is deliberately project-
and robot-agnostic: topic names, vehicle types, BT node names, and world
content are supplied by the consumer.

## Quick start

```python
from sim_harness import SimTestFixture
from sim_harness.checks import check_lidar_valid
from sim_harness.nav2 import check_lifecycle_node_active

class TestMyRobot(SimTestFixture):
    LAUNCH_PACKAGE = 'my_robot_sim'
    LAUNCH_FILE = 'sim.launch.py'

    def test_lidar(self):
        result = check_lidar_valid(self.node, '/scan')
        assert result.ok, result.details

    def test_nav2_stack(self):
        check_lifecycle_node_active(self.node, '/bt_navigator')
```

Subclass `SimTestFixture`, point it at your launch file, and use the check
functions to assert conditions without writing subscription/executor
boilerplate.

## Layers

| Layer | Module                       | Purpose                                              |
|-------|------------------------------|------------------------------------------------------|
| 0     | `sim_harness.fixture`        | `SimTestFixture` — pytest base, nodes, executor     |
| 0     | `sim_harness.spin`           | Spin helpers (`spin_for_duration`, `spin_until_*`)  |
| 0     | `sim_harness.collector`      | `MessageCollector` for topic sampling                |
| 1     | `sim_harness.checks`         | Generic sensor/service/motion checks                 |
| 1     | `sim_harness.nav2`           | Nav2 lifecycle + action checks                       |
| 1     | `sim_harness.perception`     | Detection / perception checks                        |
| 1     | `sim_harness.navigation_quality` | Motion quality tracker + pluggable monitors     |
| 2     | `sim_harness.validation`     | `RequirementValidator`, `ValidationScope`           |
| 2     | `sim_harness.simulator`      | `SimulatorInterface`, `GazeboBackend`, lifecycle    |
| 2     | `sim_harness.launch_utils`   | `chain_on_exit`, Gazebo env setup                   |

Lower layers never import higher ones. Consumers can use any layer without
the others.

## Extension points

`sim_harness` is designed to be configured, not forked. Use these hooks to
add project-specific behaviour without modifying the library.

### Scoped validation results

`RequirementValidator` records into the thread-local collector by default.
To route a fixture's results into a private `ValidationScope` (for
per-run reporting, parallel test isolation, or an external exporter):

```python
from sim_harness.validation import RequirementValidator, ValidationScope

class MyFixture(SimTestFixture, RequirementValidator):
    pass

scope = ValidationScope('suite_A')
fix = MyFixture()
fix.set_validation_scope(scope)
# ... run tests ...
scope.export_to_json('results/suite_A.json')
```

Or bind a scope at class level, or install it on the collector so that
legacy code calling `ValidationResultCollector.instance()` writes into
your scope too:

```python
from sim_harness.validation import ValidationResultCollector, ValidationScope

SUITE = ValidationScope('nightly')
ValidationResultCollector.set_scope(SUITE)
```

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
    on_timeout='fail',  # shut down the launch if it never activates
    description='Wait for bt_navigator active',
)
```

Factories returning ready-to-use condition callables:

- `topic_publishing(topic, msg_type, min_rate_hz, sample_window_sec)`
- `service_available(service_name)`
- `lifecycle_active(node_name)`

Any `Callable[[], bool]` works — roll your own if none of the factories fit.

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
├── sim_harness/            # Python package
│   ├── fixture.py          # SimTestFixture (Layer 0)
│   ├── spin.py             # Spin helpers (Layer 0)
│   ├── collector.py        # MessageCollector (Layer 0)
│   ├── checks.py           # Generic checks (Layer 1)
│   ├── nav2.py             # Nav2 checks (Layer 1)
│   ├── perception.py       # Detection checks (Layer 1)
│   ├── navigation_quality.py  # Motion tracker + monitors (Layer 1)
│   ├── validation/         # Requirement tracking (Layer 2)
│   ├── simulator/          # Simulator lifecycle (Layer 2)
│   └── launch_utils.py     # launch helpers (Layer 2)
├── src/                    # C++ library
├── include/                # Public C++ headers
├── launch/                 # Reusable launch fragments
├── test/                   # Unit tests for the harness itself
└── examples/               # Reference integrations (e.g. turtlebot3)
```

## License

Apache-2.0. See `package.xml` for dependency declarations.
