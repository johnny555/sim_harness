Custom Robot Integration
========================

Integrating sim_harness with your own robot.

Custom Fixture
--------------

.. code-block:: python

   from sim_harness import SimTestFixture

   class MyRobotTestFixture(SimTestFixture):
       LAUNCH_PACKAGE = 'my_robot_gazebo'
       LAUNCH_FILE = 'simulation.launch.py'

       SCAN_TOPIC = "/my_robot/scan"
       ODOM_TOPIC = "/my_robot/odom"

       def on_setup(self):
           self.scan = self.create_message_collector(self.SCAN_TOPIC, LaserScan)
           self.odom = self.create_message_collector(self.ODOM_TOPIC, Odometry)

Using Your Fixture
------------------

.. code-block:: python

   class TestMyRobot(MyRobotTestFixture):
       def test_sensors(self):
           self.spin_for_duration(5.0)
           assert self.scan.count() > 0

CI/CD Integration
-----------------

See the GitHub Actions example in the contributing guide.
