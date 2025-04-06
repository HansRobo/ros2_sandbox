
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <unordered_map>

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode(rclcpp::NodeOptions options)
  : Node("ament_auto_installed_package_node", options), publisher(create_publisher<example_interfaces::msg::String>("topic", 10))
  {}

private:
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
