// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#ifndef SIM_HARNESS__VALIDATION__VALIDATION_RESULT_HPP_
#define SIM_HARNESS__VALIDATION__VALIDATION_RESULT_HPP_

#include <memory>
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
 * @brief Scoped validation result collection.
 *
 * Each test suite or session can create its own scope rather than relying
 * on a process-global singleton. Scopes optionally propagate results to
 * a parent scope for hierarchical collection.
 *
 * @code
 * auto scope = std::make_shared<ValidationScope>("nav_tests");
 * scope->addResult(ValidationResult::create("REQ-001", "Nav works", true));
 * scope->exportToJson("results.json");
 * @endcode
 */
class ValidationScope
{
public:
  explicit ValidationScope(
    const std::string & name,
    std::shared_ptr<ValidationScope> parent = nullptr);

  void addResult(const ValidationResult & result);
  void clear();
  std::vector<ValidationResult> getResults() const;
  void getCounts(size_t & passed_out, size_t & failed_out) const;
  bool exportToJson(const std::string & filepath) const;
  void printSummary() const;

  const std::string & name() const { return name_; }

private:
  std::string name_;
  std::shared_ptr<ValidationScope> parent_;
  mutable std::mutex mutex_;
  std::vector<ValidationResult> results_;
};

/**
 * @brief Backward-compatible singleton collector that delegates to
 * a default ValidationScope.
 *
 * New code should prefer using ValidationScope directly. This class
 * exists so that existing code calling instance() continues to work.
 */
class ValidationResultCollector
{
public:
  static ValidationResultCollector & instance();

  void addResult(const ValidationResult & result);
  void clear();
  std::vector<ValidationResult> getResults() const;
  void getCounts(size_t & passed_out, size_t & failed_out) const;
  bool exportToJson(const std::string & filepath) const;
  void printSummary() const;

  /// Get the underlying scope.
  std::shared_ptr<ValidationScope> scope() const { return scope_; }

  /// Replace the scope (e.g., per test suite).
  void setScope(std::shared_ptr<ValidationScope> scope) { scope_ = scope; }

private:
  ValidationResultCollector();
  ValidationResultCollector(const ValidationResultCollector &) = delete;
  ValidationResultCollector & operator=(const ValidationResultCollector &) = delete;

  std::shared_ptr<ValidationScope> scope_;
};

}  // namespace sim_harness

#endif  // SIM_HARNESS__VALIDATION__VALIDATION_RESULT_HPP_
