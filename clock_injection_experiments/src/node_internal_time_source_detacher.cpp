#include "clock_injection_experiments/node_internal_time_source_detacher.hpp"

#include <memory>
#include <rclcpp/node_interfaces/node_time_source.hpp>
#include <rclcpp/time_source.hpp>
#include <stdexcept>

namespace clock_injection_experiments
{

namespace
{

// --- private-member-access トリック ---
// アクセス制御はテンプレート実引数(メンバポインタ)の検査時には適用されない、という規格の抜け穴を
// 利用して、NodeTimeSource::time_source_(private)へのメンバポインタを静的領域に保存する。
// 参考: 明示的テンプレートインスタンス化によるprivateメンバアクセス手法。
template<typename Tag>
struct stolen
{
  static typename Tag::type ptr;
};
template<typename Tag>
typename Tag::type stolen<Tag>::ptr;

template<typename Tag, typename Tag::type P>
struct steal : stolen<Tag>
{
  struct setter
  {
    setter() {stolen<Tag>::ptr = P;}
  };
  static setter setter_instance;
};
template<typename Tag, typename Tag::type P>
typename steal<Tag, P>::setter steal<Tag, P>::setter_instance;

// NodeTimeSource::time_source_ を指すメンバポインタのタグ。
struct NodeTimeSourceTimeSourceTag
{
  using type = rclcpp::TimeSource (rclcpp::node_interfaces::NodeTimeSource::*);
};
template struct steal<NodeTimeSourceTimeSourceTag,
  & rclcpp::node_interfaces::NodeTimeSource::time_source_>;

}  // namespace

void detachNodeInternalTimeSource(rclcpp::Node::SharedPtr node)
{
  // NodeTimeSourceInterface(空インターフェース)を実体のNodeTimeSourceへcastする。
  auto iface = node->get_node_time_source_interface();
  auto node_time_source =
    std::dynamic_pointer_cast<rclcpp::node_interfaces::NodeTimeSource>(iface);
  if (!node_time_source) {
    throw std::runtime_error(
            "NodeTimeSourceへのcastに失敗しました(rclcpp内部実装が想定と異なります)");
  }

  // privateメンバ time_source_ への参照を取得し、ノードのClockをdetachする。
  rclcpp::TimeSource & internal_time_source =
    (*node_time_source).*stolen<NodeTimeSourceTimeSourceTag>::ptr;
  internal_time_source.detachClock(node->get_clock());
}

}  // namespace clock_injection_experiments
