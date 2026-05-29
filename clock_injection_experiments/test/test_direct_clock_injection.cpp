// rcl APIを用いてrclcpp::Clockへ/clockトピックを経由せず時刻を直接注入する基本動作の検証。
//
// 検証の狙い:
//   rclcpp::Clock(RCL_ROS_TIME)の裏にあるrcl_clock_tをget_clock_handle()で取得し、
//   rcl_enable_ros_time_override + rcl_set_ros_time_overrideで時刻を注入すると、
//   now()が注入値と完全一致(遅延ゼロ)で返ることを確認する。

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace
{

class DirectClockInjectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// override未有効のRCL_ROS_TIME clockに対し、enable後に時刻注入するとnow()へ即時反映される。
TEST_F(DirectClockInjectionTest, InjectOnceReflectsImmediately)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * handle = clock->get_clock_handle();

  // override有効化前はROS時刻はアクティブでない。
  EXPECT_FALSE(clock->ros_time_is_active());

  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(handle)) << rcl_get_error_string().str;

  // rclcpp側ラッパー(ros_time_is_active)とrcl側getterの両層で有効化を確認する。
  EXPECT_TRUE(clock->ros_time_is_active());
  bool enabled = false;
  ASSERT_EQ(RCL_RET_OK, rcl_is_enabled_ros_time_override(handle, &enabled))
    << rcl_get_error_string().str;
  EXPECT_TRUE(enabled);

  const rcl_time_point_value_t injected = 1234567890LL;
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(handle, injected)) << rcl_get_error_string().str;

  // 遅延ゼロ・完全一致(int64比較)で注入値が反映される。
  EXPECT_EQ(clock->now().nanoseconds(), injected);

  // rcl側のgetterとも一致する。
  rcl_time_point_value_t got = 0;
  ASSERT_EQ(RCL_RET_OK, rcl_clock_get_now(handle, &got)) << rcl_get_error_string().str;
  EXPECT_EQ(got, injected);
}

// 複数回の注入が都度反映され、巻き戻し(減少)も含めて反映されることを確認する。
TEST_F(DirectClockInjectionTest, MultipleInjectionsEachReflected)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rcl_clock_t * handle = clock->get_clock_handle();

  ASSERT_EQ(RCL_RET_OK, rcl_enable_ros_time_override(handle)) << rcl_get_error_string().str;

  const std::vector<rcl_time_point_value_t> sequence = {
    1'000'000'000LL,      // 1s
    2'000'000'000LL,      // 2s
    9'000'000'000'000LL,  // 9000s (大きい値)
    500'000'000LL,        // 0.5s (巻き戻し)
  };

  for (const auto value : sequence) {
    ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(handle, value)) << rcl_get_error_string().str;
    EXPECT_EQ(clock->now().nanoseconds(), value);
  }
}

// override未有効ではROS時刻はアクティブにならず、注入のためにはenableが必須であることを固定化する。
TEST_F(DirectClockInjectionTest, NowWithoutOverrideIsNotActive)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  EXPECT_FALSE(clock->ros_time_is_active());
  // enableせずにnow()を呼んでもクラッシュしない(バッキングのシステム時刻が返る)。
  EXPECT_NO_THROW((void)clock->now());
}

// RCL_ROS_TIME以外のクロックはoverrideできないことを確認する(落とし穴の固定化)。
TEST_F(DirectClockInjectionTest, NonRosTimeClockCannotBeOverridden)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  rcl_clock_t * handle = clock->get_clock_handle();

  EXPECT_NE(RCL_RET_OK, rcl_enable_ros_time_override(handle));
  // エラー状態を残すと後続の判定に影響するためクリアする。
  rcl_reset_error();
}

}  // namespace
