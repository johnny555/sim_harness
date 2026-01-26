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

Best Practices
--------------

.. tip::

   Use consistent ID format: ``REQ-{CATEGORY}-{NUMBER}``

.. tip::

   Include measurements in details for debugging.
