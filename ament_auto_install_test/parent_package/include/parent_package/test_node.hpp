#include <example_interfaces/msg/string.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class TestNode : public rclcpp::Node
{
public:
  explicit TestNode(rclcpp::NodeOptions options)
  : Node("ament_auto_installed_package_node", options),
    publisher(create_publisher<example_interfaces::msg::String>("topic", 10))
  {
  }

private:
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher;
};