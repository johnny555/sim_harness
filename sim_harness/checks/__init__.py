# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
sim_harness.checks — Robot test checks organized by purpose.

Five buckets that match how you think about testing a robot system:

1. **readiness** — Is the system up? Nodes, services, lifecycle, controllers.
2. **sensors**   — Are sensors working? Publish rates, data validity, latency.
3. **motion**    — Does the robot move? Velocity, position, ground truth.
4. **navigation** — Can it navigate? Goals, paths, costmaps.
5. **perception** — Does it see? Object detection, classification, safety zones.

Predicate combinators (``all_of``, ``any_of``, ``negate``) live here
because they work across all buckets::

    from sim_harness.checks import all_of
    from sim_harness.checks.sensors import scan_has_min_points, scan_ranges_within

    check = all_of(scan_has_min_points(100), scan_ranges_within(0.1, 30.0))
"""

from sim_harness.core.stream_properties import (  # noqa: F401
    all_of,
    any_of,
    negate,
)
