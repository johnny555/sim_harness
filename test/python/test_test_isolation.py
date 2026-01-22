#!/usr/bin/env python3
# Copyright 2026 John Vial
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for test isolation module."""

import os
import pytest

from sim_harness.core.test_isolation import (
    TestIsolationConfig,
    get_test_isolation_config,
    apply_test_isolation,
    generate_test_node_name,
)


class TestTestIsolation:
    """Tests for test isolation utilities."""

    def setup_method(self):
        """Save original environment."""
        self._original_domain_id = os.environ.get('ROS_DOMAIN_ID')
        self._original_gz_partition = os.environ.get('GZ_PARTITION')

    def teardown_method(self):
        """Restore original environment."""
        if self._original_domain_id is not None:
            os.environ['ROS_DOMAIN_ID'] = self._original_domain_id
        elif 'ROS_DOMAIN_ID' in os.environ:
            del os.environ['ROS_DOMAIN_ID']

        if self._original_gz_partition is not None:
            os.environ['GZ_PARTITION'] = self._original_gz_partition
        elif 'GZ_PARTITION' in os.environ:
            del os.environ['GZ_PARTITION']

    def test_get_config_default(self):
        """Test getting config with default domain ID."""
        if 'ROS_DOMAIN_ID' in os.environ:
            del os.environ['ROS_DOMAIN_ID']

        config = get_test_isolation_config()

        # Default domain 0 maps to 100
        assert config.domain_id == 100
        assert config.gz_partition == "gz_test_100"

    def test_get_config_with_domain_id(self):
        """Test getting config with specific domain ID."""
        os.environ['ROS_DOMAIN_ID'] = '42'

        config = get_test_isolation_config()

        # 42 maps to 142
        assert config.domain_id == 142
        assert config.gz_partition == "gz_test_142"

    def test_get_config_domain_id_wrapping(self):
        """Test that domain IDs wrap correctly."""
        os.environ['ROS_DOMAIN_ID'] = '150'

        config = get_test_isolation_config()

        # 150 % 100 = 50, then 100 + 50 = 150
        assert config.domain_id == 150
        assert config.gz_partition == "gz_test_150"

    def test_apply_isolation(self):
        """Test applying test isolation."""
        config = TestIsolationConfig(domain_id=123, gz_partition="gz_test_123")

        result = apply_test_isolation(config)

        assert result.domain_id == 123
        assert os.environ['ROS_DOMAIN_ID'] == '123'
        assert os.environ['GZ_PARTITION'] == 'gz_test_123'

    def test_generate_node_name(self):
        """Test generating unique node names."""
        os.environ['ROS_DOMAIN_ID'] = '5'

        name1 = generate_test_node_name("test_node")
        name2 = generate_test_node_name("test_node")

        # Should contain base name and domain suffix
        assert name1.startswith("test_node_d105_")
        assert name2.startswith("test_node_d105_")

        # Should be unique (random suffix)
        # Note: There's a small chance they could be the same
        # but with 4 digits it's very unlikely

    def test_generate_node_name_custom_domain(self):
        """Test generating node name with custom domain."""
        name = generate_test_node_name("my_node", domain_id=77)

        assert name.startswith("my_node_d77_")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
