Assertion Primitives
====================

Pre-built assertion functions.

Sensor Assertions
-----------------

- ``assert_topic_published(node, topic, msg_type, timeout_sec, min_messages)``
- ``assert_sensor_data_valid(node, topic, msg_type, timeout_sec, allow_nan_ratio)``

Vehicle Assertions
------------------

- ``assert_vehicle_moved(node, vehicle_id, min_distance, velocity, timeout_sec)``
- ``assert_vehicle_stationary(node, vehicle_id, velocity_threshold, duration_sec)``
- ``assert_vehicle_velocity(node, vehicle_id, target_velocity, tolerance)``
- ``assert_vehicle_in_region(node, vehicle_id, min_bounds, max_bounds)``
- ``assert_vehicle_orientation(node, vehicle_id, expected_yaw, tolerance_rad)``

Lifecycle Assertions
--------------------

- ``assert_lifecycle_node_active(node, node_name, timeout_sec)``

Service Assertions
------------------

- ``assert_service_available(node, service_name, timeout_sec)``
- ``assert_service_call_succeeds(node, service_name, srv_type, request)``
