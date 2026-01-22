// Copyright 2026 John Vial
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "sim_harness/core/message_collector.hpp"

using sim_harness::MessageCollector;

class MessageCollectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("test_node");
    executor_.add_node(node_);
  }

  void TearDown() override
  {
    executor_.remove_node(node_);
    node_.reset();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(MessageCollectorTest, BasicCollection)
{
  auto collector = std::make_shared<MessageCollector<std_msgs::msg::String>>(
    node_, "/test_topic");

  // Create publisher
  auto pub = node_->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  // Initially no messages
  EXPECT_EQ(collector->count(), 0u);
  EXPECT_FALSE(collector->hasMessages());

  // Publish a message
  std_msgs::msg::String msg;
  msg.data = "hello";
  pub->publish(msg);

  // Spin to receive
  for (int i = 0; i < 10 && collector->count() == 0; ++i) {
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_EQ(collector->count(), 1u);
  EXPECT_TRUE(collector->hasMessages());

  auto messages = collector->getMessages();
  ASSERT_EQ(messages.size(), 1u);
  EXPECT_EQ(messages[0].data, "hello");
}

TEST_F(MessageCollectorTest, MultipleMessages)
{
  auto collector = std::make_shared<MessageCollector<std_msgs::msg::String>>(
    node_, "/test_topic");

  auto pub = node_->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  // Publish multiple messages
  for (int i = 0; i < 5; ++i) {
    std_msgs::msg::String msg;
    msg.data = "message_" + std::to_string(i);
    pub->publish(msg);
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  // Give time for all messages to arrive
  for (int i = 0; i < 20 && collector->count() < 5; ++i) {
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_GE(collector->count(), 5u);
}

TEST_F(MessageCollectorTest, GetLatest)
{
  auto collector = std::make_shared<MessageCollector<std_msgs::msg::String>>(
    node_, "/test_topic");

  // No messages yet
  EXPECT_FALSE(collector->getLatest().has_value());

  auto pub = node_->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  std_msgs::msg::String msg1, msg2;
  msg1.data = "first";
  msg2.data = "second";

  pub->publish(msg1);
  // Wait for first message
  for (int i = 0; i < 10 && collector->count() < 1; ++i) {
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  pub->publish(msg2);
  // Wait for second message
  for (int i = 0; i < 10 && collector->count() < 2; ++i) {
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  auto latest = collector->getLatest();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->data, "second");
}

TEST_F(MessageCollectorTest, Clear)
{
  auto collector = std::make_shared<MessageCollector<std_msgs::msg::String>>(
    node_, "/test_topic");

  auto pub = node_->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  std_msgs::msg::String msg;
  msg.data = "test";
  pub->publish(msg);

  for (int i = 0; i < 10 && collector->count() == 0; ++i) {
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_GT(collector->count(), 0u);

  collector->clear();

  EXPECT_EQ(collector->count(), 0u);
  EXPECT_FALSE(collector->hasMessages());
}

TEST_F(MessageCollectorTest, GetTopic)
{
  auto collector = std::make_shared<MessageCollector<std_msgs::msg::String>>(
    node_, "/my/test/topic");

  EXPECT_EQ(collector->getTopic(), "/my/test/topic");
}

TEST_F(MessageCollectorTest, Callback)
{
  auto collector = std::make_shared<MessageCollector<std_msgs::msg::String>>(
    node_, "/test_topic");

  bool callback_called = false;
  std::string received_data;

  collector->setCallback([&](const std_msgs::msg::String & msg) {
      callback_called = true;
      received_data = msg.data;
    });

  auto pub = node_->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  std_msgs::msg::String msg;
  msg.data = "callback_test";
  pub->publish(msg);

  for (int i = 0; i < 10 && !callback_called; ++i) {
    executor_.spin_some(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(callback_called);
  EXPECT_EQ(received_data, "callback_test");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
