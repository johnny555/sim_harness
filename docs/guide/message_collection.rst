Message Collection
==================

Message collectors simplify subscribing to topics and buffering messages.

Basic Usage
-----------

.. code-block:: python

   collector = self.create_message_collector("/scan", LaserScan)
   self.spin_for_duration(3.0)
   messages = collector.get_messages()
   assert len(messages) > 0

Collector Options
-----------------

.. code-block:: python

   collector = self.create_message_collector(
       "/scan",
       LaserScan,
       key="scan",           # For later retrieval
       max_messages=100,     # Buffer size limit
       qos_profile=qos       # Custom QoS
   )

Accessing Messages
------------------

.. code-block:: python

   messages = collector.get_messages()    # All messages
   latest = collector.latest()            # Most recent
   count = collector.count()              # Count
   collector.clear()                      # Clear buffer

Best Practices
--------------

.. warning::

   Don't forget to spin - collectors only receive messages when the node spins.
