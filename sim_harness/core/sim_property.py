# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Property-based testing decorators and helpers for ROS 2 simulation tests.

Provides:

- ``sim_property`` -- a pre-configured Hypothesis ``@settings`` decorator
  with defaults appropriate for simulation testing (no deadline, persistent
  database, slow health checks suppressed).

- ``check_recorded_property`` / ``check_recorded_eventually`` /
  ``check_recorded_monotonic`` -- Tier 1 helpers that check properties over
  data already collected from a sim run, without re-running the sim.

- ``hypothesis_check_recorded`` -- Tier 1 with Hypothesis: generates random
  parameters and checks them against recorded data.

Three-tier usage::

    # Tier 1 (Recorded Data) -- cheap, full Hypothesis power:
    messages = collector.messages()
    check_recorded_property(messages, lambda msg: has_valid_ranges(msg))

    # Tier 2 (Scenario-Level) -- expensive, use sparingly:
    @sim_property(max_examples=3)
    @given(goal=navigation_goal_2d(...))
    def test_reaches_goals(self, goal): ...

    # Tier 3 (Same-Sim Variation) -- medium cost, sim stays running:
    @sim_property(max_examples=10)
    @given(cmd=twist_strategy(max_linear=0.5))
    def test_velocity_commands(self, cmd): ...
"""

import os
from typing import TypeVar, Callable, List, Optional, Any

try:
    from hypothesis import settings, HealthCheck
    from hypothesis import given as hypothesis_given
    from hypothesis.database import DirectoryBasedExampleDatabase
    HAS_HYPOTHESIS = True
except ImportError:
    HAS_HYPOTHESIS = False

T = TypeVar('T')

# Default location for persistent Hypothesis example database.
_DEFAULT_DB_DIR = os.path.join(
    os.environ.get('HOME', '/tmp'),
    '.hypothesis', 'sim_harness',
)


# ---------------------------------------------------------------------------
# sim_property: pre-configured @settings for sim tests
# ---------------------------------------------------------------------------

def sim_property(
    max_examples: int = 5,
    deadline: Optional[int] = None,
    database_dir: Optional[str] = None,
    derandomize: bool = False,
    nightly: bool = False,
    **extra_settings,
):
    """
    Hypothesis ``@settings`` pre-configured for simulation property tests.

    Use this instead of bare ``@settings(...)`` to get reasonable defaults
    for sim testing:

    - No deadline (sim tests can't have deterministic timing)
    - Persistent example database (shrunk counterexamples survive across runs)
    - ``too_slow`` health check suppressed

    Args:
        max_examples: Number of test cases to generate.  Use 3-5 for
            expensive sim tests (Tier 2), 50-200 for recorded-data tests
            (Tier 1), 10-20 for same-sim variation (Tier 3).
        deadline: Per-example time limit in ms.  None (default) disables it.
        database_dir: Directory for the persistent example database.
            Defaults to ``~/.hypothesis/sim_harness/``.
        derandomize: If True, use deterministic example generation.
            Useful for CI reproducibility.
        nightly: If True, use 10x more examples for thorough nightly testing.
            Also enabled by the ``SIM_HARNESS_NIGHTLY`` environment variable.
        **extra_settings: Additional keyword args passed to
            ``hypothesis.settings``.

    Returns:
        A ``hypothesis.settings`` decorator.

    Example::

        @sim_property(max_examples=5)
        @given(goal=navigation_goal_2d(x_bounds=(-3, 3), y_bounds=(-3, 3)))
        def test_reaches_goals(self, goal):
            result = assert_reaches_goal(self.node, self.executor, goal, 30.0)
            assert result.reached
    """
    if not HAS_HYPOTHESIS:
        raise ImportError(
            "hypothesis is required for property-based testing. "
            "Install it with: pip install hypothesis"
        )

    # Check for nightly mode
    is_nightly = nightly or os.environ.get(
        'SIM_HARNESS_NIGHTLY', ''
    ).lower() in ('1', 'true', 'yes')
    if is_nightly:
        max_examples = max_examples * 10

    db_dir = database_dir or _DEFAULT_DB_DIR

    # Build suppressed health checks list
    suppressed = list(extra_settings.pop('suppress_health_check', []))
    if HealthCheck.too_slow not in suppressed:
        suppressed.append(HealthCheck.too_slow)

    return settings(
        max_examples=max_examples,
        deadline=deadline,
        database=DirectoryBasedExampleDatabase(db_dir),
        suppress_health_check=suppressed,
        derandomize=derandomize,
        **extra_settings,
    )


# ---------------------------------------------------------------------------
# PropertyFailure: descriptive assertion error with counterexample
# ---------------------------------------------------------------------------

class PropertyFailure(AssertionError):
    """
    Raised when a property check fails over recorded data.

    Attributes:
        counterexample: The specific data item that violated the property.
        failure_index: Index of the failing item in the data list.
    """

    def __init__(
        self,
        message: str,
        counterexample: Any = None,
        index: int = -1,
    ):
        self.counterexample = counterexample
        self.failure_index = index
        super().__init__(message)


# ---------------------------------------------------------------------------
# Tier 1: Property checks over recorded data (no sim re-run)
# ---------------------------------------------------------------------------

def check_recorded_property(
    data: List[T],
    property_fn: Callable[[T], bool],
    description: str = "",
    min_samples: int = 1,
) -> None:
    """
    Check that a property holds for every item in recorded data.

    This is the Tier 1 entry point: you've already collected data from
    a sim run, and now you want to check properties over it.  No sim is
    re-run -- this is as cheap as iterating a list.

    Raises ``PropertyFailure`` (subclass of ``AssertionError``) with the
    exact counterexample on failure.

    Args:
        data: List of recorded messages / data points.
        property_fn: Predicate that must hold for every item.
        description: Human-readable property description.
        min_samples: Minimum number of items required.

    Raises:
        PropertyFailure: If a counterexample is found or too few samples.

    Example::

        collector = self.create_message_collector('/scan', LaserScan)
        self.spin_for_duration(10.0)
        messages = collector.messages()

        check_recorded_property(
            messages,
            lambda scan: count_valid_points(scan) >= 100,
            description="All scans have >= 100 valid points",
        )
    """
    if len(data) < min_samples:
        raise PropertyFailure(
            f"Not enough samples: got {len(data)}, need >= {min_samples}. "
            f"Property: {description}",
        )

    for i, item in enumerate(data):
        if not property_fn(item):
            raise PropertyFailure(
                f"Property violated at index {i}/{len(data)}: {description}",
                counterexample=item,
                index=i,
            )


def check_recorded_eventually(
    data: List[T],
    property_fn: Callable[[T], bool],
    description: str = "",
) -> int:
    """
    Check that a property holds for at least one item in recorded data.

    Returns the index of the first satisfying item.

    Args:
        data: List of recorded messages.
        property_fn: Predicate that must hold for at least one item.
        description: Human-readable description.

    Returns:
        Index of the first item satisfying the property.

    Raises:
        PropertyFailure: If no item satisfies the property.
    """
    for i, item in enumerate(data):
        if property_fn(item):
            return i

    raise PropertyFailure(
        f"Property never held across {len(data)} items: {description}",
    )


def check_recorded_monotonic(
    data: List[T],
    extract: Callable[[T], float],
    strict: bool = False,
    description: str = "",
) -> None:
    """
    Check that extracted values are monotonically non-decreasing.

    Args:
        data: List of recorded messages.
        extract: Function to pull out the numeric value.
        strict: If True, values must be strictly increasing.
        description: Human-readable description.

    Raises:
        PropertyFailure: On monotonicity violation.
    """
    if len(data) < 2:
        return

    prev = extract(data[0])
    for i in range(1, len(data)):
        val = extract(data[i])
        violation = val < prev if not strict else val <= prev
        if violation:
            op = "<" if not strict else "<="
            raise PropertyFailure(
                f"Monotonicity violated at index {i}: "
                f"{val} {op} previous {prev}. {description}",
                counterexample=data[i],
                index=i,
            )
        prev = val


# ---------------------------------------------------------------------------
# Tier 1 with Hypothesis: parameterized property checking
# ---------------------------------------------------------------------------

def hypothesis_check_recorded(
    data: List[T],
    strategy: 'SearchStrategy',
    property_fn: Callable,
    description: str = "",
    max_examples: int = 100,
) -> None:
    """
    Use Hypothesis to generate random parameters and check a property
    against recorded data.

    This is the most powerful Tier 1 pattern: Hypothesis generates random
    thresholds, bounds, or parameters, and the property function checks
    each one against the entire recorded dataset.  The sim is NOT re-run
    -- only parameter generation uses Hypothesis.

    If a parameter causes failures, Hypothesis will shrink it to find the
    simplest failing value.

    Args:
        data: Recorded messages from a sim run.
        strategy: Hypothesis strategy generating parameters.
        property_fn: ``(message, param) -> bool``  Must return True if the
            message satisfies the property for the given parameter.
        description: Human-readable description.
        max_examples: Number of parameter combinations to try.

    Raises:
        PropertyFailure: If any (data[i], param) combination fails.

    Example::

        # "For any min_points in [10..500], at least 90% of LIDAR
        #  scans should have that many valid points"
        hypothesis_check_recorded(
            data=scan_messages,
            strategy=st.integers(10, 500),
            property_fn=lambda scan, min_pts: (
                sum(1 for r in scan.ranges if math.isfinite(r)) >= min_pts
            ),
            description="LIDAR scans have sufficient valid points",
            max_examples=50,
        )
    """
    if not HAS_HYPOTHESIS:
        raise ImportError(
            "hypothesis is required. Install with: pip install hypothesis"
        )

    @sim_property(max_examples=max_examples)
    @hypothesis_given(param=strategy)
    def _check(param):
        for i, item in enumerate(data):
            if not property_fn(item, param):
                raise PropertyFailure(
                    f"Property violated at data[{i}] with param={param}: "
                    f"{description}",
                    counterexample=item,
                    index=i,
                )

    _check()
