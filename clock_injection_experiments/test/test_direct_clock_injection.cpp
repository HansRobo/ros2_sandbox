// InjectionTimeSource(カスタムTimeSource)による直接時刻注入の基本動作の検証。
//
// 検証の狙い:
//   InjectionTimeSourceにRCL_ROS_TIMEのClockをattachClockすると override が有効化され、
//   injectTime()で注入した時刻が now() に完全一致(遅延ゼロ)で返ることを確認する。
//   注入は/clockトピックを経由しないため遅延が乗らない。

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <vector>

#include "clock_injection_experiments/injection_time_source.hpp"
#include "test_utils.hpp"

namespace
{

using clock_injection_experiments::InjectionTimeSource;
using clock_injection_experiments::test::ClockInjectionTestBase;

using DirectClockInjectionTest = ClockInjectionTestBase;

// attachClockでoverrideが有効化され、injectTimeした値がnow()へ即時反映される。
TEST_F(DirectClockInjectionTest, InjectAfterAttachReflectsImmediately)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // attach前はROS時刻はアクティブでない。
  EXPECT_FALSE(clock->ros_time_is_active());

  InjectionTimeSource time_source;
  time_source.attachClock(clock);

  // attachによりoverrideが有効化される。
  EXPECT_TRUE(clock->ros_time_is_active());

  const int64_t injected = 1234567890LL;
  time_source.injectTime(injected);

  // 遅延ゼロ・完全一致(int64比較)で注入値が反映される。
  EXPECT_EQ(clock->now().nanoseconds(), injected);
}

// 複数回の注入が都度反映され、巻き戻し(減少)も含めて反映されることを確認する。
TEST_F(DirectClockInjectionTest, MultipleInjectionsEachReflected)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  InjectionTimeSource time_source;
  time_source.attachClock(clock);

  const std::vector<int64_t> sequence = {
    1'000'000'000LL,      // 1s
    2'000'000'000LL,      // 2s
    9'000'000'000'000LL,  // 9000s (大きい値)
    500'000'000LL,        // 0.5s (巻き戻し)
  };

  for (const auto value : sequence) {
    time_source.injectTime(value);
    EXPECT_EQ(clock->now().nanoseconds(), value);
  }
}

// injectTime後にattachしたClockへ、最後の注入値が即反映される(InjectionTimeSource独自挙動)。
TEST_F(DirectClockInjectionTest, AttachAppliesLastInjectedTime)
{
  auto first = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  InjectionTimeSource time_source;
  time_source.attachClock(first);

  const int64_t injected = 7'000'000'000LL;  // 7s
  time_source.injectTime(injected);
  EXPECT_EQ(first->now().nanoseconds(), injected);

  // 注入後にattachした別Clockへも最後の注入値が即反映される。
  auto second = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  time_source.attachClock(second);
  EXPECT_TRUE(second->ros_time_is_active());
  EXPECT_EQ(second->now().nanoseconds(), injected);
}

// RCL_ROS_TIME以外のClockはattachできない(落とし穴の固定化)。
TEST_F(DirectClockInjectionTest, NonRosTimeClockRejected)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  InjectionTimeSource time_source;
  EXPECT_THROW(time_source.attachClock(clock), std::invalid_argument);
}

}  // namespace
