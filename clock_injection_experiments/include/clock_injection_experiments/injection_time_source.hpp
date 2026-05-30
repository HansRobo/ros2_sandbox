// /clockトピックを購読せず、rcl APIで時刻を直接注入する専用のカスタムTimeSource。
//
// rclcpp::TimeSourceが/clockを購読してClockを駆動するのに対し、InjectionTimeSourceは
// /clockを一切購読せず、injectTime()による遅延ゼロの直接注入のみでClockを駆動する。
// これにより「/clock遮断」は購読しないこと自体で構造的に達成され、ロックステップ用途で
// 再利用できる部品となる。

#ifndef CLOCK_INJECTION_EXPERIMENTS__INJECTION_TIME_SOURCE_HPP_
#define CLOCK_INJECTION_EXPERIMENTS__INJECTION_TIME_SOURCE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <vector>

namespace clock_injection_experiments
{

// /clockを購読せず直接時刻注入のみでClockを駆動するカスタムTimeSource。
class InjectionTimeSource
{
public:
  InjectionTimeSource() = default;
  ~InjectionTimeSource() = default;

  InjectionTimeSource(const InjectionTimeSource &) = delete;
  InjectionTimeSource & operator=(const InjectionTimeSource &) = delete;

  // ノードのClock(node->get_clock() = node->now()が使うClock)を管理対象に追加する便利メソッド。
  // 内部でattachClock(node->get_clock())へ委譲する。
  //
  // 注意: use_sim_time=trueのノードでは内部TimeSourceが/clockを購読しており、外部から
  // detachできない(NodeTimeSourceInterfaceは空、保持するTimeSourceはprivate、rcl APIにも
  // 購読を止める手段はない)。/clockの外乱を断つには、ノード起動時に/clockを未使用トピックへ
  // リマップする(例: --ros-args -r /clock:=/unused_clock_sink)。詳細はREADME参照。
  void attachNode(rclcpp::Node::SharedPtr node);

  // RCL_ROS_TIMEのClockを管理対象に追加し、override(ros_time_override)を有効化する。
  // RCL_ROS_TIME以外のClockが渡された場合はstd::invalid_argumentを投げる。
  // 既にinjectTime()済みなら、最後の注入値をattach時に当該Clockへ即適用する。
  void attachClock(std::shared_ptr<rclcpp::Clock> clock);

  // Clockを管理対象から外す。overrideは無効化しない
  // (rclcpp::TimeSource::detachClockと同じ挙動。注入をやめても時刻が止まるだけ)。
  void detachClock(std::shared_ptr<rclcpp::Clock> clock);

  // attach中の全Clockへ時刻を遅延ゼロで注入する(rcl_set_ros_time_override)。
  void injectTime(rcl_time_point_value_t time_ns);

  // rclcpp::Time版の薄いラッパ。
  void injectTime(const rclcpp::Time & time);

private:
  std::vector<std::shared_ptr<rclcpp::Clock>> clocks_;
  std::optional<rcl_time_point_value_t> last_injected_;
  std::mutex clock_list_lock_;
};

}  // namespace clock_injection_experiments

#endif  // CLOCK_INJECTION_EXPERIMENTS__INJECTION_TIME_SOURCE_HPP_
