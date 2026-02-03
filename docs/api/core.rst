Core API
========

The core library — 6 files providing everything needed to write simulation tests.

Library Structure
-----------------

.. code-block:: text

   sim_harness/
   ├── fixture.py      # SimTestFixture — pytest base class
   ├── collector.py     # MessageCollector — topic subscription buffer
   ├── spin.py          # spin_for_duration, spin_until_condition
   ├── assertions.py    # Sensor, timing, service, and vehicle assertions
   ├── nav2.py          # Lifecycle, controller, and navigation assertions
   └── perception.py    # Object detection and region checks

SimTestFixture
--------------

Base test fixture for ROS 2 simulation tests. Handles node lifecycle,
test isolation (unique ``ROS_DOMAIN_ID``), and optional simulation management.

**Properties:** ``node`` — the ROS 2 node, ``executor`` — the executor

**Methods:**

- ``spin_for_duration(seconds)`` — Spin the node for a duration
- ``spin_until_condition(condition, timeout_sec)`` — Spin until callable returns True
- ``create_message_collector(topic, msg_type, ...)`` — Create a managed collector
- ``get_collector(key)`` — Retrieve a collector by key
- ``clear_messages(key)`` — Clear a collector's buffer
- ``get_logger()`` — Get the node's logger
- ``on_setup()`` / ``on_teardown()`` — Override for custom lifecycle hooks

**Class Attributes (simulation control):**

- ``LAUNCH_PACKAGE`` — ROS 2 package with launch file
- ``LAUNCH_FILE`` — Launch file name
- ``LAUNCH_ARGS`` — Launch arguments dict
- ``STARTUP_TIMEOUT`` — Max startup wait (default: 60.0)
- ``REQUIRE_SIM`` — Skip if sim unavailable (default: True)
- ``USE_EXISTING_SIM`` — Connect to running sim (default: False)

``SimulationTestFixture`` is an alias for ``SimTestFixture``.

MessageCollector
----------------

Lightweight message buffer for a ROS 2 topic.

**Constructor:** ``MessageCollector(node, topic, msg_type, qos_profile=None, max_messages=None)``

**Methods:**

- ``get_messages()`` — Get all messages as a list
- ``latest()`` — Get the most recent message (or None)
- ``count()`` — Get message count
- ``clear()`` — Clear the buffer
- ``destroy()`` — Unsubscribe and clean up

Spin Helpers
------------

- ``spin_for_duration(executor, duration_sec)`` — Spin for a fixed duration
- ``spin_until_condition(executor, condition, timeout_sec)`` — Spin until condition is met
- ``spin_until_messages_received(executor, collector, count, timeout_sec)`` — Wait for N messages

Standalone Fixtures
-------------------

For tests that don't need a full ``SimTestFixture``:

.. code-block:: python

   def test_something(ros_node, ros_executor):
       # ros_node is a plain ROS 2 node
       # ros_executor is a SingleThreadedExecutor with the node attached
       pass
