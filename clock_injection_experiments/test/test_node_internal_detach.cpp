// use_sim_time=trueのノードの内部TimeSourceからnode->get_clock()をdetachし、以降は
// InjectionTimeSourceの直接注入だけでnode->now()を駆動できることを検証する。
//
// 検証の狙い:
//   detachNodeInternalTimeSource()でノード内部のTimeSourceからClockを引き剥がすと、
//   /clockが発行されてもnode->now()が上書きされなくなる(リマップを使わずコード内で遮断)。
//   これはrclcpp内部実装(NodeTimeSourceのprivateメンバ)へのアクセスに依存する上級者向け手段。

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "clock_injection_experiments/injection_time_source.hpp"
#include "clock_injection_experiments/node_internal_time_source_detacher.hpp"
#include "test_utils.hpp"

namespace
{

using clock_injection_experiments::InjectionTimeSource;
using clock_injection_experiments::detachNodeInternalTimeSource;
using clock_injection_experiments::test::ClockInjectionTestBase;
using clock_injection_experiments::test::makeSimTimeNode;
using clock_injection_experiments::test::publishClockAndSpin;

using NodeInternalDetachTest = ClockInjectionTestBase;

// 内部TimeSourceからdetach後、/clockを発行してもnode->now()が注入値のまま遮断される。
TEST_F(NodeInternalDetachTest, DetachInternalIsolatesNodeFromClockTopic)
{
  auto node = makeSimTimeNode("internal_detach_node");
  ASSERT_TRUE(node->get_clock()->ros_time_is_active());

  // 内部TimeSourceからnode->get_clock()をdetach(/clockリマップなしで遮断)。
  detachNodeInternalTimeSource(node);

  // InjectionTimeSourceで握り直して注入する(overrideは無効化されないので有効なまま)。
  InjectionTimeSource time_source;
  time_source.attachNode(node);
  const int64_t injected = 42'000'000'000LL;  // 42s
  time_source.injectTime(injected);
  EXPECT_EQ(node->now().nanoseconds(), injected);

  // 本物の/clockを発行。内部detachが効いていればnode->now()は注入値のまま。
  publishClockAndSpin(node, 99'000'000'000LL);  // 99s

  EXPECT_EQ(node->now().nanoseconds(), injected)
    << "内部detachが効かず/clockがnode->now()へ漏れている";
}

}  // namespace
