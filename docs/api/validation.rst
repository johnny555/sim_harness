Requirements Tracking API
=========================

Opt-in requirements tracking for mapping test outcomes to requirement IDs.

To use requirements tracking, add ``RequirementValidator`` as a mixin:

.. code-block:: python

   from sim_harness import SimTestFixture, RequirementValidator

   class TestWithReqs(SimTestFixture, RequirementValidator):
       def test_sensor(self):
           result = assert_lidar_valid(self.node, "/scan")
           self.assert_requirement(
               "REQ-SEN-001", "LIDAR publishes data",
               result.valid, result.details, "Sensors"
           )

RequirementValidator
--------------------

Mixin class providing requirement recording methods.

**Methods:**

- ``validate_requirement(req_id, description, passed, details, category)`` — Record
  result without failing the test
- ``assert_requirement(...)`` — Record result and raise ``AssertionError`` on failure
- ``assert_requirement_fatal(...)`` — Record result and call ``pytest.fail()`` on failure

ValidationResult
----------------

A single requirement result.

.. code-block:: python

   result = ValidationResult.create(
       requirement_id="REQ-SEN-001",
       description="LIDAR publishes data",
       passed=True,
       details="Received 100 messages",
       category="Sensors",
   )

ValidationResultCollector
-------------------------

Singleton collector for aggregating results across a test session.

**Methods:**

- ``instance()`` — Get the singleton
- ``add_result(result)`` — Add a result
- ``get_results()`` — Get all results
- ``print_summary()`` — Print pass/fail summary
- ``export_to_json(path)`` — Export to JSON file

ValidationScope
---------------

Scoped result collection for parallel-safe test suites.

**Constructor:** ``ValidationScope(name, parent=None)``

**Methods:**

- ``add_result(result)`` — Add result (propagates to parent if set)
- ``clear()`` — Clear results
- ``get_results()`` — Get all results
- ``get_counts()`` — Get (passed, failed) counts
- ``export_to_json(path)`` — Export to JSON
- ``print_summary()`` — Print summary

Scopes can be nested — results propagate to the parent:

.. code-block:: python

   parent = ValidationScope("all_tests")
   child = ValidationScope("sensor_tests", parent=parent)
   child.add_result(...)  # Also appears in parent
