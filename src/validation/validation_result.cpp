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

// ANSI color codes
constexpr const char * GREEN = "\033[92m";
constexpr const char * RED = "\033[91m";
constexpr const char * BOLD = "\033[1m";
constexpr const char * RESET = "\033[0m";

}  // namespace

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

ValidationResultCollector & ValidationResultCollector::instance()
{
  static ValidationResultCollector instance;
  return instance;
}

void ValidationResultCollector::addResult(const ValidationResult & result)
{
  std::lock_guard<std::mutex> lock(mutex_);
  results_.push_back(result);
}

void ValidationResultCollector::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  results_.clear();
}

std::vector<ValidationResult> ValidationResultCollector::getResults() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return results_;
}

void ValidationResultCollector::getCounts(size_t & passed_out, size_t & failed_out) const
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

bool ValidationResultCollector::exportToJson(const std::string & filepath) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::ofstream file(filepath);
  if (!file.is_open()) {
    return false;
  }

  size_t passed = 0, failed = 0;
  for (const auto & r : results_) {
    if (r.passed) {
      ++passed;
    } else {
      ++failed;
    }
  }

  file << "{\n";
  file << "  \"timestamp\": \"" << getCurrentTimestamp() << "\",\n";
  file << "  \"summary\": {\n";
  file << "    \"total\": " << results_.size() << ",\n";
  file << "    \"passed\": " << passed << ",\n";
  file << "    \"failed\": " << failed << "\n";
  file << "  },\n";
  file << "  \"results\": [\n";

  for (size_t i = 0; i < results_.size(); ++i) {
    const auto & r = results_[i];
    file << "    {\n";
    file << "      \"requirement_id\": \"" << escapeJson(r.requirement_id) << "\",\n";
    file << "      \"description\": \"" << escapeJson(r.description) << "\",\n";
    file << "      \"passed\": " << (r.passed ? "true" : "false") << ",\n";
    file << "      \"details\": \"" << escapeJson(r.details) << "\",\n";
    file << "      \"timestamp\": \"" << escapeJson(r.timestamp) << "\",\n";
    file << "      \"test_file\": \"" << escapeJson(r.test_file) << "\",\n";
    file << "      \"test_method\": \"" << escapeJson(r.test_method) << "\",\n";
    file << "      \"category\": \"" << escapeJson(r.category) << "\"\n";
    file << "    }";
    if (i < results_.size() - 1) {
      file << ",";
    }
    file << "\n";
  }

  file << "  ]\n";
  file << "}\n";

  return true;
}

void ValidationResultCollector::printSummary() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  size_t passed = 0, failed = 0;
  std::vector<const ValidationResult *> failed_results;

  for (const auto & r : results_) {
    if (r.passed) {
      ++passed;
    } else {
      ++failed;
      failed_results.push_back(&r);
    }
  }

  std::cout << "\n";
  std::cout << BOLD << "=== Validation Summary ===" << RESET << "\n";
  std::cout << "Total: " << results_.size() << "\n";
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

}  // namespace sim_harness
