Command Line Interface
======================

sim_harness provides a ``ros2 test`` CLI command for discovering and running
simulation tests. This integrates with the standard ros2 CLI tool.

Overview
--------

After building and sourcing the workspace, the following commands are available:

.. code-block:: bash

   ros2 test list      # List available tests
   ros2 test run       # Run tests
   ros2 test failed    # Rerun failed tests

ros2 test list
--------------

Discover and list all launch tests in the workspace.

**Usage:**

.. code-block:: bash

   ros2 test list [OPTIONS]

**Options:**

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Option
     - Description
   * - ``-p, --packages PKG [PKG ...]``
     - Filter by package name(s)
   * - ``--pattern PATTERN``
     - Glob pattern to filter tests (e.g., ``test_nav*``)
   * - ``-v, --verbose``
     - Show detailed test information including paths
   * - ``-w, --workspace DIR``
     - Workspace root directory
   * - ``--json``
     - Output in JSON format
   * - ``--no-color``
     - Disable colored output

**Examples:**

.. code-block:: bash

   # List all tests
   ros2 test list

   # List tests from a specific package
   ros2 test list -p my_robot_tests

   # List tests with full details
   ros2 test list -v

   # Filter tests by pattern
   ros2 test list --pattern "test_nav*"

   # Output as JSON (for scripting)
   ros2 test list --json

**Sample Output:**

.. code-block:: text

   Found 5 launch test(s):

   [my_robot_tests]
     test_navigation - Navigation stack integration test
     test_sensors - Sensor validation test

   [my_robot_gazebo]
     test_physics - Physics simulation validation
     test_world - World loading test
     test_spawn - Robot spawn test

ros2 test run
-------------

Run launch tests with proper isolation and Gazebo cleanup.

**Usage:**

.. code-block:: bash

   ros2 test run [TEST ...] [OPTIONS]

**Arguments:**

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Argument
     - Description
   * - ``TEST``
     - Specific test name(s) or path(s) to run (optional)

**Options:**

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Option
     - Description
   * - ``-p, --packages PKG [PKG ...]``
     - Filter by package name(s)
   * - ``--pattern PATTERN``
     - Glob pattern to filter tests
   * - ``-v, --verbose``
     - Enable verbose output (show test stdout/stderr)
   * - ``-x, --stop-on-failure``
     - Stop after first test failure
   * - ``-t, --timeout SEC``
     - Timeout per test in seconds (default: 300)
   * - ``-d, --domain-id ID``
     - Specific ROS_DOMAIN_ID to use (random if not set)
   * - ``-o, --output-dir DIR``
     - Directory for JUnit XML results
   * - ``-w, --workspace DIR``
     - Workspace root directory
   * - ``--no-sim``
     - Skip simulation tests (run unit tests only)
   * - ``--no-cleanup``
     - Skip Gazebo cleanup before/after tests
   * - ``-l, --list-only``
     - List tests that would be run without executing
   * - ``--junit-xml FILE``
     - Path for combined JUnit XML output
   * - ``--no-color``
     - Disable colored output

**Examples:**

.. code-block:: bash

   # Run all tests
   ros2 test run

   # Run a specific test
   ros2 test run test_navigation

   # Run tests from a package
   ros2 test run -p my_robot_tests

   # Run with verbose output and stop on first failure
   ros2 test run -v -x

   # Run with custom timeout
   ros2 test run -t 600

   # Run tests matching a pattern
   ros2 test run --pattern "test_nav*"

   # Preview which tests would run
   ros2 test run -l

   # Generate JUnit XML for CI
   ros2 test run --junit-xml results.xml

**Test Isolation:**

Each test runs with:

- A random ``ROS_DOMAIN_ID`` (or specified with ``-d``)
- A unique ``GZ_PARTITION`` for Gazebo isolation
- ``ROS_LOCALHOST_ONLY=1`` for network isolation

This allows tests to run without interfering with each other or other
ROS 2 processes on the system.

**Gazebo Cleanup:**

By default, the runner kills any stale Gazebo processes before and after
running tests. Use ``--no-cleanup`` to disable this behavior.

Debugging Failed Tests
----------------------

When a test fails, the default output shows only a brief summary:

.. code-block:: text

   [1/3] Running test_navigation... FAILED (5.2s)

   FAILURES:
     - test_navigation

To see **why** a test failed, use the ``-v`` (verbose) flag:

.. code-block:: bash

   # See full output for a specific failing test
   ros2 test run test_navigation -v

   # Rerun failed tests with full output
   ros2 test failed -v

With verbose mode enabled, you'll see:

- The full command being executed
- Real-time stdout/stderr from the test
- Detailed error messages and stack traces
- ROS 2 node output and logging

**Example verbose output:**

.. code-block:: text

   ============================================================
   Running: test_navigation
   Path: /home/user/ros2_ws/src/my_robot/test/test_navigation.py
   Domain ID: 142
   Command: python3 -m launch_testing.launch_test /home/user/ros2_ws/...
   ============================================================

   [INFO] [launch]: All log files can be found below...
   [INFO] [launch]: Default logging verbosity is set to INFO
   ...
   E       AssertionError: Robot did not reach goal within timeout
   ...

**Tips for debugging:**

- Use ``-v -x`` together to stop on first failure with full output
- Check ``/tmp/launch_test_results/`` for JUnit XML files with detailed results
- Use ``-d <ID>`` to set a fixed domain ID for easier debugging with ``ros2 topic``

ros2 test failed
----------------

Rerun tests that failed in the previous test run.

**Usage:**

.. code-block:: bash

   ros2 test failed [OPTIONS]

**Options:**

.. list-table::
   :widths: 30 70
   :header-rows: 1

   * - Option
     - Description
   * - ``-v, --verbose``
     - Enable verbose output
   * - ``-x, --stop-on-failure``
     - Stop after first test failure
   * - ``-t, --timeout SEC``
     - Timeout per test in seconds (default: 300)
   * - ``-d, --domain-id ID``
     - Specific ROS_DOMAIN_ID to use
   * - ``-o, --output-dir DIR``
     - Directory for JUnit XML results
   * - ``--no-cleanup``
     - Skip Gazebo cleanup before/after tests
   * - ``-l, --list-only``
     - List failed tests without running
   * - ``--clear``
     - Clear the failed tests list
   * - ``--no-color``
     - Disable colored output

**Examples:**

.. code-block:: bash

   # Rerun all failed tests
   ros2 test failed

   # List failed tests without running
   ros2 test failed -l

   # Clear the failed tests list
   ros2 test failed --clear

   # Rerun with verbose output
   ros2 test failed -v

**How It Works:**

When ``ros2 test run`` completes, it saves a list of failed tests to
``/tmp/sim_harness_failed_tests.json``. The ``ros2 test failed`` command
reads this file and reruns only those tests.

After rerunning, the failed tests list is updated to only include tests
that still fail. If all tests pass, the list is cleared automatically.

CI/CD Integration
-----------------

The CLI is designed for easy integration with CI/CD systems.

**GitHub Actions Example:**

.. code-block:: yaml

   - name: Run tests
     run: |
       source install/setup.bash
       ros2 test run --junit-xml test-results.xml

   - name: Upload test results
     uses: actions/upload-artifact@v3
     with:
       name: test-results
       path: test-results.xml

**Exit Codes:**

- ``0`` - All tests passed
- ``1`` - One or more tests failed

**JSON Output:**

Use ``ros2 test list --json`` for machine-readable test discovery:

.. code-block:: bash

   ros2 test list --json | jq '.[] | .name'

Comparison with colcon test
---------------------------

.. list-table::
   :widths: 40 30 30
   :header-rows: 1

   * - Feature
     - ros2 test
     - colcon test
   * - Test isolation
     - Automatic domain ID
     - Manual setup
   * - Gazebo cleanup
     - Automatic
     - Manual
   * - Colored output
     - Yes
     - Limited
   * - Failed test tracking
     - Built-in
     - External tools
   * - Pattern filtering
     - Yes
     - Via pytest
   * - Direct test execution
     - Yes
     - Via launch_testing

The ``ros2 test`` command is designed specifically for simulation tests
and provides better isolation and cleanup than generic test runners.
