cmake_minimum_required(VERSION 3.8)
project(sim_harness)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Find dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Used only by the standalone ekf_wrapper_node executable below.
find_package(rclcpp REQUIRED)
find_package(robot_localization REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# =============================================================================
# Python package
# =============================================================================
# setup.cfg registers the pytest11 entry point (sim_harness_jama) which
# auto-loads @pytest.mark.requirement and the --jama-xlsx flag whenever
# sim_harness is on the import path.
ament_python_install_package(${PROJECT_NAME})

# Python bridge nodes for gz_ros2_control + Nav2 integration.
install(PROGRAMS
  scripts/tf_relay.py
  scripts/twist_to_twist_stamped.py
  scripts/gps_map_odom_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# C++ helper: ekf_wrapper_node
# =============================================================================
# Standalone executable — starts the rclcpp executor before robot_localization's
# initialize() to prevent a sim-time deadlock. Used as an EKF alternative in
# articulated_nav2_stack.launch.py.
add_executable(ekf_wrapper_node src/simulator/ekf_wrapper_node.cpp)
ament_target_dependencies(ekf_wrapper_node
  rclcpp robot_localization
  diagnostic_updater diagnostic_msgs std_srvs
  geometry_msgs nav_msgs sensor_msgs
  tf2 tf2_ros tf2_geometry_msgs)
target_include_directories(ekf_wrapper_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(ekf_wrapper_node ${robot_localization_LIBRARIES})

install(TARGETS ekf_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

# =============================================================================
# Launch + ament index
# =============================================================================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# =============================================================================
# Testing — pytest only (the C++ test library was deleted)
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(test_message_collector_py test/python/test_message_collector.py)
  ament_add_pytest_test(test_launch_wait_py test/python/test_launch_wait.py)

  add_launch_test(
    test/integration/test_turtlebot3_integration.launch.py
    TIMEOUT 300
  )
endif()

ament_package()
