// TimeSourceにattachしたClockをdetachClockで/clockから引き剥がし、
// 以降は直接注入のみが効き/clock発行の影響を受けないことを検証する。
//
// 検証の狙い(ロックステップ用途):
//   1. use_sim_time=true + TimeSource + attachClockで/clock経由の更新が効くことを確認。
//   2. detachClock後にrcl APIで時刻を直接注入する。
//   3. その後/clockを別の値で発行してもnow()が直接注入値のまま影響を受けないことを確認。

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time_source.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace
{

using namespace std::chrono_literals;

class ClockIsolationTest : public ::testing::Test
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

TEST_F(ClockIsolationTest, ClockTopicDrivesThenIsolatedAfterDetach)
{
  // use_sim_time=trueでノードを生成(TimeSourceがoverrideを有効化する)。
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<rclcpp::Node>("clock_isolation_test_node", options);

  auto ros_clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // use_clock_thread=falseで内部スレッドを無効化し、自前executorのspinに直列化する
  // (直接注入とのスレッド競合を避ける)。
  rclcpp::TimeSource time_source(node, rclcpp::ClockQoS(), /*use_clock_thread=*/false);
  time_source.attachClock(ros_clock);

  // /clockを発行する側のノード。
  auto pub_node = std::make_shared<rclcpp::Node>("test_clock_publisher");
  auto clock_pub =
    pub_node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(pub_node);

  // 指定値を/clockに発行し、ros_clockへ反映されるまでspinしながら待つ。
  auto publish_and_wait = [&](int64_t ns) -> bool {
      rosgraph_msgs::msg::Clock msg;
      msg.clock = rclcpp::Time(ns, RCL_ROS_TIME);
      const auto deadline = std::chrono::steady_clock::now() + 5s;
      while (std::chrono::steady_clock::now() < deadline) {
        clock_pub->publish(msg);
      // spin_someがイベント到着待ちを兼ねるため、別途のsleepは不要。
        executor.spin_some(50ms);
        if (ros_clock->now().nanoseconds() == ns) {
          return true;
        }
      }
      return false;
    };

  // --- ステップ1: /clock経由でros_clockが更新されることを確認 ---
  ASSERT_TRUE(ros_clock->ros_time_is_active());
  const int64_t kTimeViaTopic = 5'000'000'000LL;  // 5s
  ASSERT_TRUE(publish_and_wait(kTimeViaTopic)) << "/clock経由でClockが更新されなかった";
  EXPECT_EQ(ros_clock->now().nanoseconds(), kTimeViaTopic);

  // --- ステップ2: TimeSourceからClockを引き剥がす ---
  time_source.detachClock(ros_clock);
  // detachしてもoverride自体は無効化されない(TimeSourceはdisableを呼ばない)。
  EXPECT_TRUE(ros_clock->ros_time_is_active());

  // --- ステップ3: rcl APIで時刻を直接注入 ---
  // ステップ2でoverrideが有効なまま(detachはdisableしない)なので、enableの再確認は不要。
  // use_clock_thread=falseかつ自前spinと同一スレッドで注入するためロックも不要。
  rcl_clock_t * handle = ros_clock->get_clock_handle();
  const int64_t kInjectedTime = 42'000'000'000LL;  // 42s
  ASSERT_EQ(RCL_RET_OK, rcl_set_ros_time_override(handle, kInjectedTime))
    << rcl_get_error_string().str;
  EXPECT_EQ(ros_clock->now().nanoseconds(), kInjectedTime);

  // --- ステップ4: /clockを別値で発行しても直接注入値が保持されること ---
  const int64_t kNoiseTime = 99'000'000'000LL;  // 99s
  rosgraph_msgs::msg::Clock noise;
  noise.clock = rclcpp::Time(kNoiseTime, RCL_ROS_TIME);
  for (int i = 0; i < 3; ++i) {
    clock_pub->publish(noise);
    executor.spin_some(50ms);
  }
  EXPECT_EQ(ros_clock->now().nanoseconds(), kInjectedTime) << "detach後に/clockがClockへ漏れている";
}

}  // namespace
