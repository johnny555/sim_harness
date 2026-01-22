// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__VALIDATION__VALIDATION_RESULT_HPP_
#define SIM_HARNESS__VALIDATION__VALIDATION_RESULT_HPP_

#include <mutex>
#include <string>
#include <vector>

namespace sim_harness
{

/**
 * @brief Stores the result of a single requirement validation.
 *
 * Used for requirements traceability - mapping test results back to
 * specification requirements.
 */
struct ValidationResult
{
  /// Requirement ID (e.g., "REQ-001", "Matt-SYSRQ-52")
  std::string requirement_id;

  /// Human-readable description of the requirement
  std::string description;

  /// Whether the requirement was satisfied
  bool passed;

  /// Additional details about the validation
  std::string details;

  /// ISO 8601 timestamp of when the validation occurred
  std::string timestamp;

  /// Source file where the test was defined
  std::string test_file;

  /// Test method name
  std::string test_method;

  /// Category for grouping (e.g., "Vehicle Data", "Sensors", "Navigation")
  std::string category;

  /**
   * @brief Create a ValidationResult with auto-generated timestamp.
   *
   * @param req_id Requirement ID
   * @param desc Description
   * @param passed Whether passed
   * @param details Optional details
   * @param category Optional category
   * @return ValidationResult with current timestamp
   */
  static ValidationResult create(
    const std::string & req_id,
    const std::string & desc,
    bool passed,
    const std::string & details = "",
    const std::string & category = "");
};

/**
 * @brief Singleton collector for validation results.
 *
 * Accumulates validation results across all tests and provides
 * export functionality for traceability reports.
 */
class ValidationResultCollector
{
public:
  /**
   * @brief Get the singleton instance.
   *
   * @return Reference to the global collector
   */
  static ValidationResultCollector & instance();

  /**
   * @brief Add a validation result.
   *
   * Thread-safe.
   *
   * @param result The result to add
   */
  void addResult(const ValidationResult & result);

  /**
   * @brief Clear all collected results.
   *
   * Thread-safe.
   */
  void clear();

  /**
   * @brief Get all collected results.
   *
   * Thread-safe.
   *
   * @return Copy of all results
   */
  std::vector<ValidationResult> getResults() const;

  /**
   * @brief Get count of passed/failed results.
   *
   * @param passed_out Output: number of passed validations
   * @param failed_out Output: number of failed validations
   */
  void getCounts(size_t & passed_out, size_t & failed_out) const;

  /**
   * @brief Export results to a JSON file.
   *
   * JSON format:
   * @code
   * {
   *   "timestamp": "2024-01-15T10:30:00",
   *   "summary": {"total": 10, "passed": 8, "failed": 2},
   *   "results": [...]
   * }
   * @endcode
   *
   * @param filepath Path to write JSON file
   * @return true if export succeeded
   */
  bool exportToJson(const std::string & filepath) const;

  /**
   * @brief Print a summary to stdout.
   *
   * Includes pass/fail counts and list of failed requirements.
   */
  void printSummary() const;

private:
  ValidationResultCollector() = default;
  ValidationResultCollector(const ValidationResultCollector &) = delete;
  ValidationResultCollector & operator=(const ValidationResultCollector &) = delete;

  mutable std::mutex mutex_;
  std::vector<ValidationResult> results_;
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__VALIDATION__VALIDATION_RESULT_HPP_
