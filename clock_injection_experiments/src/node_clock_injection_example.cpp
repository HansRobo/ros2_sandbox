// InjectionTimeSourceでrclcpp::Node::now()を駆動するデモ。
//
// ノードのClock(node->get_clock())をInjectionTimeSourceにattachNodeし、ループで時刻を注入
// しながら node->now() がその注入値に追従することを表示する。/clockトピックは使わない。
//
// use_sim_time=trueでノード全体をsim timeモードにしつつ、/clockを未使用トピックへリマップして
// 内部TimeSourceの購読を空振りさせ、本物の/clockによる外乱を遮断している。
//
// 実行例(ros2 runは自動でプログラム名を補うのでリマップ指定はそのまま渡せる):
//   ros2 run clock_injection_experiments node_clock_injection_example \
//     --ros-args -p use_sim_time:=true -r /clock:=/unused_clock_sink

#include <chrono>
#include <cstdint>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "clock_injection_experiments/injection_time_source.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // use_sim_time=trueでノードを生成する。これによりノード全体(タイマー・TF等)がsim timeで
  // 動作し、node->get_clock()のros_time_overrideは内部TimeSourceにより既に有効化されている。
  // このデモは/clockを発行しないので、注入値はそのまま保たれる。実環境で本物の/clockの外乱を
  // 断つには、起動時に -r /clock:=/unused_clock_sink を渡す(use_global_argumentsで自動反映)か、
  // detachNodeInternalTimeSource(node)を使う(README参照)。
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});
  auto node = std::make_shared<rclcpp::Node>("node_clock_injection_example", options);

  // ノードのClockをInjectionTimeSourceに設定する(use_sim_time=trueなので既にoverride有効)。
  clock_injection_experiments::InjectionTimeSource time_source;
  time_source.attachNode(node);

  RCLCPP_INFO(
    node->get_logger(),
    "InjectionTimeSourceでnode->now()を駆動します(use_sim_time=true, /clockはリマップで遮断)");

  // 1秒刻みの仮想時刻を順に注入し、node->now()が追従することを表示する。
  int64_t virtual_time_ns = 0;
  for (int step = 0; step < 5 && rclcpp::ok(); ++step) {
    time_source.injectTime(virtual_time_ns);

    RCLCPP_INFO(
      node->get_logger(),
      "injected = %.3f s, node->now() = %.3f s",
      virtual_time_ns / 1e9,
      node->now().seconds());

    virtual_time_ns += 1'000'000'000LL;  // +1s
  }

  rclcpp::shutdown();
  return 0;
}
