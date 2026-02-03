# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Nav2 extensions — lifecycle management, navigation goals, and path assertions.

Use these when testing a robot with the ROS 2 Navigation stack (Nav2),
managed lifecycle nodes, or ros2_control controllers.

Example::

    from sim_harness import SimTestFixture
    from sim_harness.nav2 import (
        assert_nav2_active,
        assert_reaches_goal,
    )

    class TestNavigation(SimTestFixture):
        LAUNCH_PACKAGE = 'my_robot_nav'
        LAUNCH_FILE = 'nav.launch.py'

        def test_nav_stack_up(self):
            result = assert_nav2_active(self.node)
            assert result.success, result.details

        def test_reaches_goal(self):
            result = assert_reaches_goal(self.node, x=1.0, y=0.0)
            assert result.success, result.details
"""

# --- Lifecycle / controller assertions ---
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

# --- Navigation assertions ---
from sim_harness.primitives.navigation_assertions import (  # noqa: F401
    NavigationResult,
    assert_reaches_goal,
    assert_follows_path,
    assert_navigation_action_succeeds,
    assert_costmap_contains_obstacle,
)
