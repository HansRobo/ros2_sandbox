// Copyright 2025 Kotaro Yoshimoto, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <sample_interfaces/msg/test.hpp>
#include <sample_interfaces/msg/test_a.hpp>
#include <sample_interfaces/msg/test_b.hpp>

class TestNode : public rclcpp::Node
{
public:
  TestNode() : Node("test_node", rclcpp::NodeOptions())
  {
    publisher = create_publisher<sample_interfaces::msg::Test>("test", 10);
    publisher_a = create_publisher<sample_interfaces::msg::TestA>("test_a", 10);
    publisher_b = create_publisher<sample_interfaces::msg::TestB>("test_b", 10);

    timer = create_wall_timer(std::chrono::milliseconds(100), [&]() {
      sample_interfaces::msg::Test msg;
      msg.a = 1;
      msg.b = 2;
      msg.c = 3;
      msg.d = 4;
      publisher->publish(msg);

      sample_interfaces::msg::TestA msg_a;
      msg_a.name = "test_node";
      msg_a.value = 42;
      msg_a.timestamp = this->now().seconds();
      publisher_a->publish(msg_a);

      sample_interfaces::msg::TestB msg_b;
      msg_b.flag = true;
      msg_b.data = {1, 2, 3, 4, 5};
      msg_b.description = "Test message B";
      publisher_b->publish(msg_b);
    });
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<sample_interfaces::msg::Test>::SharedPtr publisher;
  rclcpp::Publisher<sample_interfaces::msg::TestA>::SharedPtr publisher_a;
  rclcpp::Publisher<sample_interfaces::msg::TestB>::SharedPtr publisher_b;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
