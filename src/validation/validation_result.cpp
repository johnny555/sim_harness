// Copyright 2026 The sim_harness Authors
// SPDX-License-Identifier: Apache-2.0

#include "sim_harness/validation/validation_result.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace sim_harness
{

namespace
{

std::string getCurrentTimestamp()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S");
  return ss.str();
}

// Simple JSON string escaping
std::string escapeJson(const std::string & s)
{
  std::stringstream ss;
  for (char c : s) {
    switch (c) {
      case '"': ss << "\\\""; break;
      case '\\': ss << "\\\\"; break;
      case '\n': ss << "\\n"; break;
      case '\r': ss << "\\r"; break;
      case '\t': ss << "\\t"; break;
      default: ss << c; break;
    }
  }
  return ss.str();
}

void writeResultsJson(
  std::ostream & out,
  const std::string & scope_name,
  const std::vector<ValidationResult> & results)
{
  size_t passed = 0, failed = 0;
  for (const auto & r : results) {
    if (r.passed) {
      ++passed;
    } else {
      ++failed;
    }
  }

  out << "{\n";
  if (!scope_name.empty()) {
    out << "  \"scope\": \"" << escapeJson(scope_name) << "\",\n";
  }
  out << "  \"timestamp\": \"" << getCurrentTimestamp() << "\",\n";
  out << "  \"summary\": {\n";
  out << "    \"total\": " << results.size() << ",\n";
  out << "    \"passed\": " << passed << ",\n";
  out << "    \"failed\": " << failed << "\n";
  out << "  },\n";
  out << "  \"results\": [\n";

  for (size_t i = 0; i < results.size(); ++i) {
    const auto & r = results[i];
    out << "    {\n";
    out << "      \"requirement_id\": \"" << escapeJson(r.requirement_id) << "\",\n";
    out << "      \"description\": \"" << escapeJson(r.description) << "\",\n";
    out << "      \"passed\": " << (r.passed ? "true" : "false") << ",\n";
    out << "      \"details\": \"" << escapeJson(r.details) << "\",\n";
    out << "      \"timestamp\": \"" << escapeJson(r.timestamp) << "\",\n";
    out << "      \"test_file\": \"" << escapeJson(r.test_file) << "\",\n";
    out << "      \"test_method\": \"" << escapeJson(r.test_method) << "\",\n";
    out << "      \"category\": \"" << escapeJson(r.category) << "\"\n";
    out << "    }";
    if (i < results.size() - 1) {
      out << ",";
    }
    out << "\n";
  }

  out << "  ]\n";
  out << "}\n";
}

void printSummaryImpl(
  const std::string & label,
  const std::vector<ValidationResult> & results)
{
  constexpr const char * GREEN = "\033[92m";
  constexpr const char * RED = "\033[91m";
  constexpr const char * BOLD = "\033[1m";
  constexpr const char * RESET = "\033[0m";

  size_t passed = 0, failed = 0;
  std::vector<const ValidationResult *> failed_results;

  for (const auto & r : results) {
    if (r.passed) {
      ++passed;
    } else {
      ++failed;
      failed_results.push_back(&r);
    }
  }

  std::cout << "\n";
  std::cout << BOLD << "=== Validation Summary";
  if (!label.empty()) {
    std::cout << " (" << label << ")";
  }
  std::cout << " ===" << RESET << "\n";
  std::cout << "Total: " << results.size() << "\n";
  std::cout << GREEN << "Passed: " << passed << RESET << "\n";
  std::cout << RED << "Failed: " << failed << RESET << "\n";

  if (!failed_results.empty()) {
    std::cout << "\n" << BOLD << "Failed Requirements:" << RESET << "\n";
    for (const auto * r : failed_results) {
      std::cout << RED << "  [FAIL] " << RESET
                << r->requirement_id << ": " << r->description << "\n";
      if (!r->details.empty()) {
        std::cout << "         Details: " << r->details << "\n";
      }
    }
  }

  std::cout << "\n";
}

}  // namespace

// ---------------------------------------------------------------------------
// ValidationResult
// ---------------------------------------------------------------------------

ValidationResult ValidationResult::create(
  const std::string & req_id,
  const std::string & desc,
  bool passed,
  const std::string & details,
  const std::string & category)
{
  ValidationResult result;
  result.requirement_id = req_id;
  result.description = desc;
  result.passed = passed;
  result.details = details;
  result.timestamp = getCurrentTimestamp();
  result.category = category;
  return result;
}

// ---------------------------------------------------------------------------
// ValidationScope
// ---------------------------------------------------------------------------

ValidationScope::ValidationScope(
  const std::string & name,
  std::shared_ptr<ValidationScope> parent)
: name_(name), parent_(parent)
{
}

void ValidationScope::addResult(const ValidationResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  results_.push_back(result);
  // Propagate to parent while holding our lock. This prevents race conditions
  // when multiple threads add results to the same child scope simultaneously.
  // Note: This creates a lock ordering constraint - parent locks must be
  // acquired after child locks to avoid deadlock.
  if (parent_) {
    parent_->addResult(result);
  }
}

void ValidationScope::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  results_.clear();
}

std::vector<ValidationResult> ValidationScope::getResults() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return results_;
}

void ValidationScope::getCounts(size_t & passed_out, size_t & failed_out) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  passed_out = 0;
  failed_out = 0;
  for (const auto & r : results_) {
    if (r.passed) {
      ++passed_out;
    } else {
      ++failed_out;
    }
  }
}

bool ValidationScope::exportToJson(const std::string & filepath) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::ofstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  writeResultsJson(file, name_, results_);
  return true;
}

void ValidationScope::printSummary() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  printSummaryImpl(name_, results_);
}

// ---------------------------------------------------------------------------
// ValidationResultCollector (backward-compatible delegate)
// ---------------------------------------------------------------------------

ValidationResultCollector::ValidationResultCollector()
: scope_(std::make_shared<ValidationScope>("default"))
{
}

ValidationResultCollector & ValidationResultCollector::instance()
{
  static ValidationResultCollector instance;
  return instance;
}

void ValidationResultCollector::addResult(const ValidationResult & result)
{
  scope_->addResult(result);
}

void ValidationResultCollector::clear()
{
  scope_->clear();
}

std::vector<ValidationResult> ValidationResultCollector::getResults() const
{
  return scope_->getResults();
}

void ValidationResultCollector::getCounts(size_t & passed_out, size_t & failed_out) const
{
  scope_->getCounts(passed_out, failed_out);
}

bool ValidationResultCollector::exportToJson(const std::string & filepath) const
{
  return scope_->exportToJson(filepath);
}

void ValidationResultCollector::printSummary() const
{
  scope_->printSummary();
}

}  // namespace sim_harness
