cmake_minimum_required(VERSION 3.8)
project(sim_harness)

# Compiler settings
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
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(cpp_dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_action
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_msgs
  lifecycle_msgs
  controller_manager_msgs
  nav2_msgs
  tf2_ros
)

# =============================================================================
# C++ Library
# =============================================================================
add_library(${PROJECT_NAME} SHARED
  # Core
  src/core/test_fixture_base.cpp
  src/core/test_isolation.cpp
  # Validation
  src/validation/validation_result.cpp
  # Primitives
  src/primitives/vehicle_assertions.cpp
  src/primitives/sensor_assertions.cpp
  src/primitives/lifecycle_assertions.cpp
  src/primitives/service_assertions.cpp
  src/primitives/navigation_assertions.cpp
  src/primitives/perception_assertions.cpp
  src/primitives/timing_assertions.cpp
  # Simulator
  src/simulator/gazebo_backend.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${cpp_dependencies})

# =============================================================================
# Python Module
# =============================================================================
ament_python_install_package(sim_harness)

# Install Python scripts/entry points
install(PROGRAMS
  sim_harness/test_runner/cli.py
  DESTINATION lib/${PROJECT_NAME}
)

# Note: setup.cfg contains entry points for ros2cli and is automatically
# processed by setuptools when ament_python_install_package builds the
# Python package. The ros2 test command and its verbs will be registered
# with ros2cli after building and sourcing the workspace.

# =============================================================================
# Install
# =============================================================================

# Install C++ headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install C++ library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

# Install resource marker for ament index
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# =============================================================================
# Examples (optional, built by default)
# =============================================================================
option(BUILD_EXAMPLES "Build example integration tests" ON)

if(BUILD_EXAMPLES)
  find_package(ament_cmake_gtest REQUIRED)

  # TurtleBot3 integration test example
  add_executable(turtlebot3_integration_test
    examples/turtlebot3_integration_test.cpp
  )
  target_link_libraries(turtlebot3_integration_test
    ${PROJECT_NAME}
    gtest
    gtest_main
  )
  ament_target_dependencies(turtlebot3_integration_test ${cpp_dependencies})

  install(TARGETS turtlebot3_integration_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # ----- C++ Unit Tests -----
  ament_add_gtest(test_validation_result_cpp test/cpp/test_validation_result.cpp)
  target_link_libraries(test_validation_result_cpp ${PROJECT_NAME})
  ament_target_dependencies(test_validation_result_cpp ${cpp_dependencies})

  ament_add_gtest(test_message_collector_cpp test/cpp/test_message_collector.cpp)
  target_link_libraries(test_message_collector_cpp ${PROJECT_NAME})
  ament_target_dependencies(test_message_collector_cpp ${cpp_dependencies})

  # ----- Python Unit Tests -----
  ament_add_pytest_test(test_validation_result_py test/python/test_validation_result.py)
  ament_add_pytest_test(test_message_collector_py test/python/test_message_collector.py)


  # ----- Integration Tests (launch_testing) -----
  add_launch_test(
    test/integration/test_turtlebot3_integration.launch.py
    TIMEOUT 300
  )
endif()

# =============================================================================
# Export
# =============================================================================
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(${cpp_dependencies})

ament_package()
