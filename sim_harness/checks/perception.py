# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Perception checks — "Does the robot see the world?"

Validates object detection, classification, counting,
and safety-zone clearance.
"""

# Re-export everything from the implementation module.
from sim_harness.primitives.perception_assertions import (  # noqa: F401
    DetectionResult,
    assert_object_detected,
    assert_object_detected_by_class,
    assert_min_objects_detected,
    assert_region_clear,
)
