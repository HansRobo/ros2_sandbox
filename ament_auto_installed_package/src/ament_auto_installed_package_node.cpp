
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ament_auto_installed_package/test_node.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
