Core API
========

Core classes and functions for sim_harness.

SimTestFixture
--------------

Base test fixture for ROS 2 simulation tests.

**Properties:** ``node`` - The ROS 2 node instance

**Methods:**

- ``get_logger()`` - Get the node's logger
- ``spin_for_duration(seconds)`` - Spin the node
- ``create_message_collector(topic, msg_type, ...)`` - Create collector
- ``assert_requirement(...)`` - Record and assert requirement
- ``validate_requirement(...)`` - Record without asserting

SimulationTestFixture
---------------------

Test fixture with automatic simulation lifecycle.

**Class Attributes:**

- ``LAUNCH_PACKAGE`` - ROS 2 package with launch file
- ``LAUNCH_FILE`` - Launch file name
- ``LAUNCH_ARGS`` - Launch arguments dict
- ``STARTUP_TIMEOUT`` - Max startup wait (default: 60.0)

MessageCollector
----------------

Collects messages from a ROS 2 topic.

**Methods:**

- ``get_messages()`` - Get all messages
- ``get_latest()`` - Get most recent message
- ``message_count()`` - Get count
- ``clear()`` - Clear buffer
- ``wait_for_messages(count, timeout)`` - Wait for messages

ValidationResultCollector
-------------------------

Singleton collector for validation results.

**Methods:**

- ``instance()`` - Get singleton
- ``add_result(result)`` - Add result
- ``get_results()`` - Get all results
- ``print_summary()`` - Print summary
- ``export_to_json(path)`` - Export to JSON
