
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unordered_map>

class MultiDomainTalker
{
public:
  MultiDomainTalker(int argc, char ** argv, int domain_id_1, int domain_id_2)
  {
    {
      context1 = std::make_shared<rclcpp::Context>();
      rclcpp::InitOptions init_options;
      init_options.set_domain_id(domain_id_1);
      context1->init(argc, argv, init_options);

      node1 = std::make_shared<rclcpp::Node>("node_1", rclcpp::NodeOptions().context(context1));
      publisher1 = node1->create_publisher<std_msgs::msg::String>("topic", 10);
      RCLCPP_INFO(node1->get_logger(), "Node 1 initialized.");

      rclcpp::ExecutorOptions executor_options;
      executor_options.context = context1;
      executor1 = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
      executor1->add_node(node1);
    }

    {
      context2 = std::make_shared<rclcpp::Context>();
      rclcpp::InitOptions init_options;
      init_options.set_domain_id(domain_id_2);
      context2->init(argc, argv, init_options);

      node2 = std::make_shared<rclcpp::Node>("node_2", rclcpp::NodeOptions().context(context2));
      publisher2 = node2->create_publisher<std_msgs::msg::String>("topic", 10);
      RCLCPP_INFO(node2->get_logger(), "Node 2 initialized.");

      rclcpp::ExecutorOptions executor_options;
      executor_options.context = context2;
      executor2 = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);
      executor2->add_node(node2);
    }

    rclcpp::install_signal_handlers();
  }

  void publish1(const std::string & message)
  {
    std_msgs::msg::String msg;
    msg.data = message;
    publisher1->publish(msg);
  }

  void publish2(const std::string & message)
  {
    std_msgs::msg::String msg;
    msg.data = message;
    publisher2->publish(msg);
  }

  void spin_some()
  {
    executor1->spin_some();
    executor2->spin_some();
  }

  bool ok() { return context1->is_valid() && context2->is_valid(); }

  bool shutdown()
  {
    bool ret1 = rclcpp::shutdown(context1);
    bool ret2 = rclcpp::shutdown(context2);
    rclcpp::uninstall_signal_handlers();
    return ret1 && ret2;
  }

private:
  rclcpp::Context::SharedPtr context1;
  rclcpp::Context::SharedPtr context2;
  rclcpp::Node::SharedPtr node1;
  rclcpp::Node::SharedPtr node2;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher1;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor1;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor2;
};

int main(int argc, char ** argv)
{
  auto multi_domain_talker = std::make_shared<MultiDomainTalker>(argc, argv, 0, 1);
  rclcpp::Rate rate(2);
  while (multi_domain_talker->ok()) {
    multi_domain_talker->publish1("Hello World from domain 0");
    multi_domain_talker->publish2("Hello World from domain 1");
    multi_domain_talker->spin_some();
    rate.sleep();
  }

  multi_domain_talker->shutdown();
  return 0;
}
