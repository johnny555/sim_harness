# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Perception extensions — object detection and region checks.

Use these when testing computer-vision or detection pipelines
that publish detection messages.

Example::

    from sim_harness import SimTestFixture
    from sim_harness.perception import (
        assert_object_detected,
        assert_region_clear,
    )

    class TestPerception(SimTestFixture):
        def test_detects_obstacle(self):
            result = assert_object_detected(
                self.node, '/detections', 'obstacle')
            assert result.success, result.details
"""

from sim_harness.primitives.perception_assertions import (  # noqa: F401
    DetectionResult,
    assert_object_detected,
    assert_object_detected_by_class,
    assert_min_objects_detected,
    assert_region_clear,
)
