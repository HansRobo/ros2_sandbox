// 【上級者向け・実装依存】use_sim_time=trueのノードの内部TimeSourceから、ノードのClockを
// detachするユーティリティ。
//
// 通常は/clockを未使用トピックへリマップする方法(堅牢・実装非依存)を推奨する。本ユーティリティは
// 起動引数を使わずコード内で完結させたい場合の代替手段だが、rclcpp内部のNodeTimeSource実装
// (privateメンバ time_source_)に依存するため、ROSバージョン更新で壊れる可能性がある。

#ifndef CLOCK_INJECTION_EXPERIMENTS__NODE_INTERNAL_TIME_SOURCE_DETACHER_HPP_
#define CLOCK_INJECTION_EXPERIMENTS__NODE_INTERNAL_TIME_SOURCE_DETACHER_HPP_

#include <rclcpp/node.hpp>

namespace clock_injection_experiments
{

// ノードの内部TimeSourceから node->get_clock() をdetachし、/clockによる上書きを止める。
//
// これにより use_sim_time=true のノードでも、以降は /clock の影響を受けず InjectionTimeSource の
// 直接注入だけで node->now() を駆動できる(overrideは無効化されないので時刻は保たれる)。
//
// 注意: rclcpp内部のNodeTimeSource実装に依存する。castに失敗した場合はstd::runtime_errorを投げる。
void detachNodeInternalTimeSource(rclcpp::Node::SharedPtr node);

}  // namespace clock_injection_experiments

#endif  // CLOCK_INJECTION_EXPERIMENTS__NODE_INTERNAL_TIME_SOURCE_DETACHER_HPP_
