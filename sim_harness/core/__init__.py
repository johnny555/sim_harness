# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Core test infrastructure for ROS 2 simulation testing.

Provides the test fixture, message collection, spin helpers,
topic observation, composable predicates, and readiness checks.
"""

from sim_harness.core.test_fixture import SimTestFixture
from sim_harness.core.message_collector import MessageCollector
from sim_harness.core.spin_helpers import (
    spin_for_duration,
    spin_until_condition,
    spin_until_messages_received,
)
from sim_harness.core.test_isolation import (
    get_test_isolation_config,
    apply_test_isolation,
    generate_test_node_name,
    TestIsolationConfig,
)
from sim_harness.core.readiness_check import (
    ReadinessCheck,
    CheckResult,
    CheckItemResult,
    CheckStatus,
    CheckCategory,
    create_standard_check,
)
from sim_harness.core.topic_observer import (
    TopicObserver,
    ObservationResult,
    ParallelObserver,
    SENSOR_QOS,
    collect_messages,
    count_messages,
    latest_message,
    track_max,
    track_timestamps,
)
from sim_harness.core.predicates import (
    all_of,
    any_of,
    negate,
)

__all__ = [
    'SimTestFixture',
    'MessageCollector',
    'spin_for_duration',
    'spin_until_condition',
    'spin_until_messages_received',
    'get_test_isolation_config',
    'apply_test_isolation',
    'generate_test_node_name',
    'TestIsolationConfig',
    # Readiness checks
    'ReadinessCheck',
    'CheckResult',
    'CheckItemResult',
    'CheckStatus',
    'CheckCategory',
    'create_standard_check',
    # TopicObserver
    'TopicObserver',
    'ObservationResult',
    'ParallelObserver',
    'SENSOR_QOS',
    'collect_messages',
    'count_messages',
    'latest_message',
    'track_max',
    'track_timestamps',
    # Predicate combinators
    'all_of',
    'any_of',
    'negate',
]
