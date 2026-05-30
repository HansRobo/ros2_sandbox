// テスト間で共通するセットアップ(init/shutdown、use_sim_timeノード生成、/clockノイズ発行)を
// まとめたヘルパ。各テストはこれを使ってボイラープレートを避ける。

#ifndef CLOCK_INJECTION_EXPERIMENTS__TEST_UTILS_HPP_
#define CLOCK_INJECTION_EXPERIMENTS__TEST_UTILS_HPP_

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <string>

namespace clock_injection_experiments::test
{

using namespace std::chrono_literals;

// rclcpp::init/shutdownを行う共通テスト基底クラス。
class ClockInjectionTestBase : public ::testing::Test
{
protected:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};

// use_sim_time=true でノードを生成する。
inline std::shared_ptr<rclcpp::Node> makeSimTimeNode(const std::string & name)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  return std::make_shared<rclcpp::Node>(name, options);
}

// /clockに指定時刻をcount回発行し、その都度node_under_testを含むexecutorをspinする。
// (注入値が/clockの外乱で上書きされないことを確認するテスト用)
inline void publishClockAndSpin(
  const rclcpp::Node::SharedPtr & node_under_test, int64_t time_ns, int count = 5)
{
  auto pub_node = std::make_shared<rclcpp::Node>("test_clock_publisher");
  auto clock_pub =
    pub_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_under_test);
  executor.add_node(pub_node);

  rosgraph_msgs::msg::Clock msg;
  msg.clock = rclcpp::Time(time_ns, RCL_ROS_TIME);
  for (int i = 0; i < count; ++i) {
    clock_pub->publish(msg);
    executor.spin_some(50ms);
  }
}

}  // namespace clock_injection_experiments::test

#endif  // CLOCK_INJECTION_EXPERIMENTS__TEST_UTILS_HPP_
