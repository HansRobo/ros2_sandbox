
#include <memory>
#include <parent_package/test_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
