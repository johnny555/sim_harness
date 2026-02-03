# Copyright 2026 The sim_harness Authors
# SPDX-License-Identifier: Apache-2.0

"""
Sensor checks — "Are sensors producing good data?"

Validates that sensors publish at expected rates, that data is sane
(no NaN, within physical bounds), and that timing/latency is acceptable.
Includes composable predicates for building custom sensor validation.

Example::

    from sim_harness.checks.sensors import (
        assert_lidar_valid,
        assert_publish_rate,
        scan_has_min_points,
        scan_ranges_within,
    )

    class TestSensors(SimTestFixture):
        def test_lidar(self):
            result = assert_lidar_valid(self.node, '/scan')
            assert result.valid, result.details

        def test_custom_scan_check(self):
            from sim_harness.checks import all_of
            check = all_of(scan_has_min_points(200), scan_ranges_within(0.1, 30.0))
            # use with TopicObserver or recorded data
"""

# --- Sensor data assertions ---
from sim_harness.primitives.sensor_assertions import (  # noqa: F401
    SensorDataResult,
    assert_sensor_publishing,
    assert_lidar_valid,
    assert_gps_valid,
    assert_imu_valid,
    assert_camera_valid,
    assert_joint_states_valid,
)

# --- Timing assertions (publish rate, latency, TF) ---
from sim_harness.primitives.timing_assertions import (  # noqa: F401
    TimingResult,
    assert_publish_rate,
    assert_latency,
    assert_transform_available,
)

# --- Composable sensor predicates ---
from sim_harness.core.predicates import (  # noqa: F401
    # LIDAR / LaserScan
    scan_has_min_points,
    scan_ranges_within,
    scan_nan_ratio_below,
    scan_has_range_count,
    # IMU
    imu_accel_within,
    imu_gyro_within,
    imu_no_nan,
    # Camera / Image
    image_has_data,
    image_dimensions,
    image_encoding,
    # GPS / NavSatFix
    gps_no_nan,
    gps_in_bounds,
    gps_has_fix,
    # Joint states
    joints_present,
    joints_no_nan,
)
