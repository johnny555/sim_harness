/*
 * EKF Wrapper Node
 *
 * Wraps robot_localization's RosEkf to fix sim_time clock deadlock.
 *
 * The stock ekf_node calls initialize() before spin(), causing
 * wait_until_started() to block. This wrapper spins first, then initializes.
 *
 * use_sim_time is passed via node-level args (not global) to avoid SIGSEGV
 * during construction. The node-level TimeSource subscribes to /clock and
 * the executor delivers callbacks before initialize() is called.
 */
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "robot_localization/ros_filter_types.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Forward all CLI args at node level, adding use_sim_time explicitly.
  std::vector<std::string> node_args;
  node_args.push_back("ekf_filter_node");
  for (int i = 1; i < argc; i++) {
    node_args.push_back(argv[i]);
  }

  rclcpp::NodeOptions options;
  options.arguments(node_args);
  options.use_global_arguments(false);
  // Explicitly set use_sim_time as a parameter override so the TimeSource
  // activates its /clock subscription during construction.
  options.append_parameter_override("use_sim_time", true);

  auto filter = std::make_shared<robot_localization::RosEkf>(options);

  // Start executor BEFORE initialize() so /clock callbacks are delivered.
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(filter->get_node_base_interface());

  std::thread spin_thread([executor]() {
    executor->spin();
  });

  // Wait for sim clock to be received.
  RCLCPP_INFO(filter->get_logger(), "Waiting for sim clock...");
  auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (std::chrono::steady_clock::now() < deadline) {
    if (filter->get_clock()->started()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (filter->get_clock()->started()) {
    RCLCPP_INFO(filter->get_logger(),
      "Sim clock active (t=%.1fs), initializing EKF",
      filter->get_clock()->now().seconds());
  } else {
    RCLCPP_WARN(filter->get_logger(), "No sim clock after 30s, proceeding anyway");
  }

  filter->initialize();

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
