/*
 * EKF Wrapper Node
 *
 * Wraps robot_localization's RosEkf to fix a clock initialization deadlock.
 *
 * The stock ekf_node calls initialize() before spin(). When use_sim_time
 * is true, initialize() blocks on get_clock()->wait_until_started() waiting
 * for /clock messages. The TimeSource clock thread should deliver these,
 * but in complex launch scenarios with many nodes, DDS discovery delays
 * can cause an indefinite hang.
 *
 * This wrapper starts the executor BEFORE calling initialize(), ensuring
 * /clock messages are delivered promptly via the executor thread.
 */
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "robot_localization/ros_filter_types.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  options.clock_type(RCL_ROS_TIME);

  auto filter = std::make_shared<robot_localization::RosEkf>(options);

  // Start the executor BEFORE calling initialize().
  // This ensures /clock subscription callbacks are processed, so
  // wait_until_started() resolves promptly instead of deadlocking.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(filter->get_node_base_interface());

  std::thread spin_thread([executor]() {
    executor->spin();
  });

  filter->initialize();

  // Block until SIGINT triggers rclcpp::shutdown() → executor stops
  spin_thread.join();

  rclcpp::shutdown();
  return 0;
}
