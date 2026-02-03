Requirement Tracing
===================

Map tests to requirements for traceability and compliance reporting.

Basic Usage
-----------

.. code-block:: python

   self.assert_requirement(
       req_id="REQ-SEN-001",
       description="LIDAR sensor publishes scan data",
       passed=True,
       details="Received 150 messages",
       category="Sensors"
   )

Methods
-------

- ``assert_requirement()`` - Records and asserts (fails if passed=False)
- ``validate_requirement()`` - Records without asserting

Validation Summary
------------------

.. code-block:: python

   from sim_harness import ValidationResultCollector

   collector = ValidationResultCollector.instance()
   collector.print_summary()
   collector.export_to_json("/tmp/results.json")

ValidationScope
---------------

For parallel-safe result collection, use ``ValidationScope`` instead of the
global singleton:

.. code-block:: python

   from sim_harness import ValidationScope, ValidationResult

   scope = ValidationScope("navigation_tests")
   scope.add_result(ValidationResult.create("REQ-001", "Nav works", True))
   scope.export_to_json("/tmp/nav_results.json")

Scopes can be nested -- results propagate to the parent:

.. code-block:: python

   parent = ValidationScope("all_tests")
   child = ValidationScope("sensor_tests", parent=parent)
   child.add_result(...)  # Also appears in parent

Best Practices
--------------

.. tip::

   Use consistent ID format: ``REQ-{CATEGORY}-{NUMBER}``

.. tip::

   Include measurements in details for debugging.

.. tip::

   Use ``ValidationScope`` instead of ``ValidationResultCollector.instance()``
   for parallel test safety.
