//
// Created by hans on 24/10/27.
//

#include <chrono>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include "timer_experiments/timer_statistics.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::vector<rclcpp::Parameter> params = { rclcpp::Parameter("use_sim_time", true) };
  rclcpp::NodeOptions node_options;
  node_options.parameter_overrides(params);

  auto node = std::make_shared<rclcpp::Node>("normal_timer", node_options);

  TimerStatistics timer_statistics(1000);


  auto start_time = node->get_clock()->now();

  auto callback = [&]() {
    timer_statistics.record_now();
    std::cout << (node->get_clock()->now() - start_time).seconds() << std::endl;
    static int count = 0;
    if (count++ > 10) {
      auto durations = timer_statistics.to_durations();
      // export to csv
      std::ofstream file("normal_timer.csv", std::ios::out);
      for (auto duration : durations) {
        file << duration.count() << std::endl;
        std::cout << duration.count() << std::endl;
      }

      // calculate statistics
      double sum = 0;
      for (auto duration : durations) {
        sum += duration.count();
      }
      double mean = sum / durations.size();
      double variance = 0;
      for (auto duration : durations) {
        variance += (duration.count() - mean) * (duration.count() - mean);
      }
      variance /= durations.size();
      double std_dev = sqrt(variance);
      std::cout << "mean: " << mean << std::endl;
      std::cout << "variance: " << variance << std::endl;
      std::cout << "std_dev: " << std_dev << std::endl;

      file.close();
      rclcpp::shutdown();
    }
  };

  auto ros_timer_ =
    rclcpp::create_timer(node, node->get_clock(), std::chrono::milliseconds(30), callback);

  std::cout << "clock_type: " << node->get_clock()->get_clock_type() << std::endl;

  // wall_timerなので、sim_timeがtrueでもwall_timerはwall timeを使う
  // auto wall_timer = node->create_wall_timer(
  //     std::chrono::milliseconds(30),callback
  // );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  return 0;
}
