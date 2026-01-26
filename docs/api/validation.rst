Validation API
==============

Classes for requirement validation.

ValidationResult
----------------

A single validation result.

.. code-block:: python

   result = ValidationResult(
       req_id="REQ-SEN-001",
       description="LIDAR publishes data",
       passed=True,
       details="Received 100 messages",
       category="Sensors",
   )

ValidationResultCollector
-------------------------

Singleton collector for all results.

.. code-block:: python

   collector = ValidationResultCollector.instance()
   collector.add_result(result)
   collector.print_summary()
   collector.export_to_json("/tmp/results.json")
   collector.export_to_csv("/tmp/results.csv")
