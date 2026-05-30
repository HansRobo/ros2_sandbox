// rclcpp::Node::now()が使うClock(node->get_clock())をInjectionTimeSourceに設定し、
// 注入した時刻がnode->now()へ反映されることを検証する。
//
// 検証の狙い(ロックステップ用途):
//   ロックステップではノード全体(タイマー・TF等)をsim timeで動かすため use_sim_time=true で
//   ノードを生成する。この状態でノード内部のTimeSourceがnode->get_clock()のoverrideを
//   既に有効化しているので、InjectionTimeSourceでattachNode(node)し injectTime() すれば、
//   /clockトピックを経由せず遅延ゼロで node->now() を駆動できる。
//
// /clock外乱の遮断について(検証で確認した制約):
//   ノード内部のTimeSourceは外部からdetachできない(NodeTimeSourceInterfaceは空、保持する
//   TimeSourceはprivate、rcl APIにも購読を止める手段はない)。そのため/clockが実際に発行されると
//   内部TimeSourceがnode->now()を上書きする。外乱を断つには、ノード起動時に/clockを未使用
//   トピックへリマップして内部TimeSourceの購読を空振りさせる(RemapIsolatesNodeFromClockTopic)。

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "clock_injection_experiments/injection_time_source.hpp"
#include "test_utils.hpp"

namespace
{

using clock_injection_experiments::InjectionTimeSource;
using clock_injection_experiments::test::ClockInjectionTestBase;
using clock_injection_experiments::test::makeSimTimeNode;
using clock_injection_experiments::test::publishClockAndSpin;

using NodeClockInjectionTest = ClockInjectionTestBase;

// use_sim_time=trueのノードでは、内部TimeSourceにより生成時点でoverrideが有効化されている。
TEST_F(NodeClockInjectionTest, SimTimeActivatesRosTimeOnConstruction)
{
  auto node = makeSimTimeNode("sim_time_activation_node");

  // attachする前から ros_time_is_active() は true(内部TimeSourceが有効化済み)。
  EXPECT_TRUE(node->get_clock()->ros_time_is_active());
}

// attachNode(node)でnode->get_clock()を握り、injectTimeした値がnode->now()へ即時反映される。
TEST_F(NodeClockInjectionTest, NodeNowReflectsInjectedTime)
{
  auto node = makeSimTimeNode("node_clock_injection_test_node");

  // attachNodeはnode->get_clock()をattachClockする。use_sim_time=trueなので既に有効化済み。
  InjectionTimeSource time_source;
  time_source.attachNode(node);
  EXPECT_TRUE(node->get_clock()->ros_time_is_active());

  const int64_t injected = 123'000'000'000LL;  // 123s
  time_source.injectTime(injected);

  // node->now()もnode->get_clock()->now()も注入値を返す(遅延ゼロ)。
  EXPECT_EQ(node->now().nanoseconds(), injected);
  EXPECT_EQ(node->get_clock()->now().nanoseconds(), injected);
}

// 複数回の注入が都度node->now()へ反映され、巻き戻しも反映されることを確認する。
TEST_F(NodeClockInjectionTest, RepeatedInjectionReflectedOnNodeNow)
{
  auto node = makeSimTimeNode("node_clock_repeated_injection_node");

  InjectionTimeSource time_source;
  time_source.attachNode(node);

  const std::vector<int64_t> sequence = {
    1'000'000'000LL,   // 1s
    10'000'000'000LL,  // 10s
    3'000'000'000LL,   // 3s (巻き戻し)
  };

  for (const auto value : sequence) {
    time_source.injectTime(value);
    EXPECT_EQ(node->now().nanoseconds(), value);
  }
}

// /clockを未使用トピックへリマップすると、内部TimeSourceの購読が空振りし外乱を遮断できる。
// これがuse_sim_time=trueを保ちつつ/clock外乱を断つ実用解(内部TimeSourceはdetach不可)。
TEST_F(NodeClockInjectionTest, RemapIsolatesNodeFromClockTopic)
{
  // 起動引数で/clockを未使用トピックへリマップする(先頭はプログラム名プレースホルダ)。
  rclcpp::NodeOptions options;
  options.arguments(
    {"remap_isolation_node", "--ros-args", "-r", "/clock:=/unused_clock_sink"});
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<rclcpp::Node>("remap_isolation_node", options);

  EXPECT_TRUE(node->get_clock()->ros_time_is_active());

  InjectionTimeSource time_source;
  time_source.attachNode(node);
  const int64_t injected = 42'000'000'000LL;  // 42s
  time_source.injectTime(injected);
  EXPECT_EQ(node->now().nanoseconds(), injected);

  // 本物の/clockを発行する。リマップにより内部TimeSourceには届かない。
  publishClockAndSpin(node, 99'000'000'000LL);  // 99s

  // node->now()は注入値のまま(/clockの外乱を構造的に遮断できている)。
  EXPECT_EQ(node->now().nanoseconds(), injected)
    << "リマップしたのに/clockがnode->now()へ漏れている";
}

}  // namespace
