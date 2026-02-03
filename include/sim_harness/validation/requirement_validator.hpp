// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__VALIDATION__REQUIREMENT_VALIDATOR_HPP_
#define SIM_HARNESS__VALIDATION__REQUIREMENT_VALIDATOR_HPP_

#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "sim_harness/validation/validation_result.hpp"

namespace sim_harness
{

namespace detail
{
// ANSI color codes for console output
constexpr const char * GREEN = "\033[92m";
constexpr const char * RED = "\033[91m";
constexpr const char * BLUE = "\033[94m";
constexpr const char * BOLD = "\033[1m";
constexpr const char * RESET = "\033[0m";
}  // namespace detail

/**
 * @brief Mixin providing requirement validation methods.
 *
 * Add this to your test class to enable requirement tracking:
 *
 * @code
 * class MyTest : public TestFixtureBase,
 *                public RequirementValidator {
 *   // ...
 * };
 *
 * TEST_F(MyTest, SomeTest) {
 *   assertRequirement("REQ-001", "System does X", condition);
 * }
 * @endcode
 */
class RequirementValidator
{
public:
  /**
   * @brief Validate a requirement (non-fatal).
   *
   * Records the validation result and prints to console, but does not
   * fail the test if the requirement is not met.
   *
   * @param req_id Requirement ID (e.g., "REQ-001")
   * @param description Human-readable description
   * @param passed Whether the requirement was satisfied
   * @param details Optional additional details
   * @param category Optional category for grouping
   * @return The passed value (for chaining)
   */
  bool validateRequirement(
    const std::string & req_id,
    const std::string & description,
    bool passed,
    const std::string & details = "",
    const std::string & category = "")
  {
    // Get test info if available
    std::string test_file;
    std::string test_method;
    if (const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info()) {
      test_file = test_info->file() ? test_info->file() : "";
      test_method = test_info->name() ? test_info->name() : "";
    }

    // Create and record result
    ValidationResult result = ValidationResult::create(
      req_id, description, passed, details, category);
    result.test_file = test_file;
    result.test_method = test_method;

    ValidationResultCollector::instance().addResult(result);

    // Print to console
    if (passed) {
      std::cout << detail::GREEN << "[PASS]" << detail::RESET
                << " " << req_id << ": " << description << "\n";
    } else {
      std::cout << detail::RED << "[FAIL]" << detail::RESET
                << " " << req_id << ": " << description << "\n";
      if (!details.empty()) {
        std::cout << "       Details: " << details << "\n";
      }
    }

    return passed;
  }

  /**
   * @brief Assert a requirement (fatal on failure).
   *
   * Records the validation result and fails the test if the requirement
   * is not met.
   *
   * @param req_id Requirement ID
   * @param description Human-readable description
   * @param condition The condition to assert
   * @param details Optional additional details
   * @param category Optional category for grouping
   */
  void assertRequirement(
    const std::string & req_id,
    const std::string & description,
    bool condition,
    const std::string & details = "",
    const std::string & category = "")
  {
    // Record the validation
    bool passed = validateRequirement(req_id, description, condition, details, category);

    // Fail the test if not met
    EXPECT_TRUE(passed)
      << "Requirement " << req_id << " not met: " << description
      << (details.empty() ? "" : "\nDetails: " + details);
  }

  /**
   * @brief Assert a requirement with ASSERT (stops test on failure).
   *
   * Like assertRequirement, but uses ASSERT_TRUE which stops the test
   * immediately on failure.
   *
   * @param req_id Requirement ID
   * @param description Human-readable description
   * @param condition The condition to assert
   * @param details Optional additional details
   * @param category Optional category for grouping
   */
  void assertRequirementFatal(
    const std::string & req_id,
    const std::string & description,
    bool condition,
    const std::string & details = "",
    const std::string & category = "")
  {
    // Record the validation
    bool passed = validateRequirement(req_id, description, condition, details, category);

    // Fail the test immediately if not met
    ASSERT_TRUE(passed)
      << "Requirement " << req_id << " not met: " << description
      << (details.empty() ? "" : "\nDetails: " + details);
  }
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__VALIDATION__REQUIREMENT_VALIDATOR_HPP_
