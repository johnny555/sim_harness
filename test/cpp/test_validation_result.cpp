// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>

#include "sim_harness/validation/validation_result.hpp"

using sim_harness::ValidationResult;
using sim_harness::ValidationResultCollector;

class ValidationResultTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ValidationResultCollector::instance().clear();
  }

  void TearDown() override
  {
    ValidationResultCollector::instance().clear();
  }
};

TEST_F(ValidationResultTest, CreateValidationResult)
{
  auto result = ValidationResult::create(
    "REQ-001",
    "Test requirement",
    true,
    "Some details",
    "TestCategory");

  EXPECT_EQ(result.requirement_id, "REQ-001");
  EXPECT_EQ(result.description, "Test requirement");
  EXPECT_TRUE(result.passed);
  EXPECT_EQ(result.details, "Some details");
  EXPECT_EQ(result.category, "TestCategory");
  EXPECT_FALSE(result.timestamp.empty());
}

TEST_F(ValidationResultTest, CollectorAddAndGet)
{
  auto & collector = ValidationResultCollector::instance();

  EXPECT_EQ(collector.getResults().size(), 0u);

  collector.addResult(ValidationResult::create("REQ-001", "Test 1", true));
  collector.addResult(ValidationResult::create("REQ-002", "Test 2", false));
  collector.addResult(ValidationResult::create("REQ-003", "Test 3", true));

  auto results = collector.getResults();
  EXPECT_EQ(results.size(), 3u);
  EXPECT_EQ(results[0].requirement_id, "REQ-001");
  EXPECT_EQ(results[1].requirement_id, "REQ-002");
  EXPECT_EQ(results[2].requirement_id, "REQ-003");
}

TEST_F(ValidationResultTest, CollectorCounts)
{
  auto & collector = ValidationResultCollector::instance();

  collector.addResult(ValidationResult::create("REQ-001", "Test 1", true));
  collector.addResult(ValidationResult::create("REQ-002", "Test 2", false));
  collector.addResult(ValidationResult::create("REQ-003", "Test 3", true));
  collector.addResult(ValidationResult::create("REQ-004", "Test 4", false));

  size_t passed, failed;
  collector.getCounts(passed, failed);

  EXPECT_EQ(passed, 2u);
  EXPECT_EQ(failed, 2u);
}

TEST_F(ValidationResultTest, CollectorClear)
{
  auto & collector = ValidationResultCollector::instance();

  collector.addResult(ValidationResult::create("REQ-001", "Test 1", true));
  EXPECT_EQ(collector.getResults().size(), 1u);

  collector.clear();
  EXPECT_EQ(collector.getResults().size(), 0u);
}

TEST_F(ValidationResultTest, ExportToJson)
{
  auto & collector = ValidationResultCollector::instance();

  collector.addResult(ValidationResult::create("REQ-001", "Test 1", true, "detail1", "Cat1"));
  collector.addResult(ValidationResult::create("REQ-002", "Test 2", false, "detail2", "Cat2"));

  std::string filepath = "/tmp/test_validation_results.json";
  ASSERT_TRUE(collector.exportToJson(filepath));

  // Verify file exists and contains expected content
  std::ifstream file(filepath);
  ASSERT_TRUE(file.is_open());

  std::string content((std::istreambuf_iterator<char>(file)),
    std::istreambuf_iterator<char>());

  EXPECT_TRUE(content.find("REQ-001") != std::string::npos);
  EXPECT_TRUE(content.find("REQ-002") != std::string::npos);
  EXPECT_TRUE(content.find("\"passed\": true") != std::string::npos);
  EXPECT_TRUE(content.find("\"passed\": false") != std::string::npos);
  EXPECT_TRUE(content.find("\"total\": 2") != std::string::npos);

  // Cleanup
  std::filesystem::remove(filepath);
}

TEST_F(ValidationResultTest, SingletonBehavior)
{
  auto & collector1 = ValidationResultCollector::instance();
  auto & collector2 = ValidationResultCollector::instance();

  collector1.addResult(ValidationResult::create("REQ-001", "Test", true));

  // Both references should see the same data
  EXPECT_EQ(collector2.getResults().size(), 1u);
  EXPECT_EQ(&collector1, &collector2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
