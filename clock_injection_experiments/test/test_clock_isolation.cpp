// InjectionTimeSourceが/clockを購読しないことによる「構造的遮断」を検証する。
//
// 検証の狙い(ロックステップ用途):
//   InjectionTimeSourceにattachしたClockは/clockを一切購読しないため、injectTime()で
//   注入した時刻が、背景で/clockに別の値が発行され続けても上書きされない(構造的に遮断)。
//   標準のrclcpp::TimeSourceでは detachClock で能動的に引き剥がす必要があったが、
//   InjectionTimeSourceでは「そもそも購読しない」設計により遮断が前提として成立する。

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "clock_injection_experiments/injection_time_source.hpp"
#include "test_utils.hpp"

namespace
{

using clock_injection_experiments::InjectionTimeSource;
using clock_injection_experiments::test::ClockInjectionTestBase;
using clock_injection_experiments::test::makeSimTimeNode;
using clock_injection_experiments::test::publishClockAndSpin;

using ClockIsolationTest = ClockInjectionTestBase;

TEST_F(ClockIsolationTest, InjectedClockIsImmuneToClockTopic)
{
  // use_sim_time=trueのノード(/clock配信が有効になる典型的なロックステップ環境)をspin対象に使う。
  auto node = makeSimTimeNode("clock_isolation_test_node");

  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // InjectionTimeSourceは/clockを購読しない。attachでoverrideが有効化される。
  InjectionTimeSource time_source;
  time_source.attachClock(ros_clock);
  ASSERT_TRUE(ros_clock->ros_time_is_active());

  // --- ステップ1: injectTimeで時刻を直接注入し、now()へ即時反映されることを確認 ---
  const int64_t kInjectedTime = 42'000'000'000LL;  // 42s
  time_source.injectTime(kInjectedTime);
  EXPECT_EQ(ros_clock->now().nanoseconds(), kInjectedTime);

  // --- ステップ2: /clockを別値で発行し続けても注入値が保持されること ---
  // InjectionTimeSourceは/clockを購読しないため、外乱はClockへ到達しない(構造的遮断)。
  publishClockAndSpin(node, 99'000'000'000LL);  // 99s
  EXPECT_EQ(ros_clock->now().nanoseconds(), kInjectedTime)
    << "/clockがClockへ漏れている(購読していないはず)";

  // --- ステップ3: detachClock後もoverrideは無効化されないことを確認 ---
  // (rclcpp::TimeSourceと同じ挙動。注入をやめても時刻が止まるだけで巻き戻らない)。
  time_source.detachClock(ros_clock);
  EXPECT_TRUE(ros_clock->ros_time_is_active());
  EXPECT_EQ(ros_clock->now().nanoseconds(), kInjectedTime);
}

}  // namespace
