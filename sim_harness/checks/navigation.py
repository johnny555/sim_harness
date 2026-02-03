# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Navigation checks — "Can the robot navigate to goals?"

Validates goal-reaching, path-following, costmap obstacles,
and Nav2 action server integration.
"""

# Re-export everything from the implementation module.
# The actual code lives in primitives/ until we remove that layer.
from sim_harness.primitives.navigation_assertions import (  # noqa: F401
    NavigationResult,
    assert_reaches_goal,
    assert_follows_path,
    assert_navigation_action_succeeds,
    assert_costmap_contains_obstacle,
)
