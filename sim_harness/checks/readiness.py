# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Readiness checks — "Is the system up and running?"

Everything you need to verify that nodes, services, lifecycle nodes,
controllers, and navigation stacks are alive and in the expected state.
Run these first before testing any robot behaviour.

Example::

    from sim_harness.checks.readiness import (
        assert_nav2_active,
        assert_node_running,
        assert_service_available,
    )

    class TestSystemReady(SimTestFixture):
        def test_nav2_stack(self):
            results = assert_nav2_active(self.node)
            assert all(r.success for r in results)

        def test_lidar_node(self):
            assert assert_node_running(self.node, "rplidar_node")
"""

# --- Node and service availability (from service_assertions) ---
from sim_harness.primitives.service_assertions import (  # noqa: F401
    ServiceResult,
    assert_service_available,
    assert_action_server_available,
    assert_node_running,
    assert_nodes_running,
    assert_parameter_exists,
)

# --- Lifecycle and controller state (from lifecycle_assertions) ---
from sim_harness.primitives.lifecycle_assertions import (  # noqa: F401
    LifecycleState,
    LifecycleResult,
    ControllerResult,
    LocalizationResult,
    lifecycle_state_to_string,
    assert_lifecycle_node_active,
    assert_lifecycle_node_state,
    assert_lifecycle_nodes_active,
    assert_controller_active,
    assert_controllers_active,
    assert_controller_manager_available,
    assert_nav2_active,
    assert_slam_toolbox_active,
    assert_localization_active,
)

# --- Timing-based readiness (from timing_assertions) ---
from sim_harness.primitives.timing_assertions import (  # noqa: F401
    assert_action_server_responsive,
)
