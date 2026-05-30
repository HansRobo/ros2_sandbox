#include "clock_injection_experiments/injection_time_source.hpp"

#include <algorithm>
#include <rcl/error_handling.h>
#include <stdexcept>
#include <string>

namespace clock_injection_experiments
{

namespace
{

// rcl APIの戻り値を検査し、失敗時はエラー文字列付きでstd::runtime_errorを投げる。
void check_rcl(rcl_ret_t ret, const char * api_name)
{
  if (ret != RCL_RET_OK) {
    const std::string msg = rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(std::string(api_name) + "に失敗しました: " + msg);
  }
}

}  // namespace

void InjectionTimeSource::attachNode(rclcpp::Node::SharedPtr node)
{
  // node->now()が使うのはnode->get_clock()。これをattachすれば node->now() も注入値を返す。
  attachClock(node->get_clock());
}

void InjectionTimeSource::attachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  std::lock_guard<std::mutex> lock(clock_list_lock_);

  rcl_clock_t * handle = clock->get_clock_handle();

  // RCL_ROS_TIME以外はrcl_enable_ros_time_overrideが失敗する(注入できない)ため、
  // attach時点で弾いて落とし穴を不変条件として固定する。
  if (handle->type != RCL_ROS_TIME) {
    throw std::invalid_argument(
            "InjectionTimeSourceにはRCL_ROS_TIMEのClockのみattachできます");
  }

  // overrideは一度有効化されればdetachClockでは無効化されない。重複enableを避けるため
  // 現在の状態を確認してから有効化する。
  bool enabled = false;
  check_rcl(
    rcl_is_enabled_ros_time_override(handle, &enabled), "rcl_is_enabled_ros_time_override");
  if (!enabled) {
    check_rcl(rcl_enable_ros_time_override(handle), "rcl_enable_ros_time_override");
  }

  clocks_.push_back(clock);

  // 既に注入済みなら、新規attachされたClockへも最後の注入値を即反映する。
  if (last_injected_.has_value()) {
    check_rcl(
      rcl_set_ros_time_override(handle, last_injected_.value()), "rcl_set_ros_time_override");
  }
}

void InjectionTimeSource::detachClock(std::shared_ptr<rclcpp::Clock> clock)
{
  std::lock_guard<std::mutex> lock(clock_list_lock_);

  // 管理対象から外すのみ。overrideは無効化しない(rclcpp::TimeSourceと同じ挙動)。
  auto it = std::remove(clocks_.begin(), clocks_.end(), clock);
  clocks_.erase(it, clocks_.end());
}

void InjectionTimeSource::injectTime(rcl_time_point_value_t time_ns)
{
  std::lock_guard<std::mutex> lock(clock_list_lock_);

  // 後からattachされるClockにも反映できるよう、最後の注入値を保持する。
  last_injected_ = time_ns;

  for (const auto & clock : clocks_) {
    check_rcl(
      rcl_set_ros_time_override(clock->get_clock_handle(), time_ns), "rcl_set_ros_time_override");
  }
}

void InjectionTimeSource::injectTime(const rclcpp::Time & time)
{
  injectTime(time.nanoseconds());
}

}  // namespace clock_injection_experiments
