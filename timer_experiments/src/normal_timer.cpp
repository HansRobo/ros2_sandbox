//
// Created by hans on 24/10/27.
//

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <fstream>

#include "timer_experiments/timer_statistics.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("normal_timer");

    TimerStatistics timer_statistics(1000);

    auto timer = node->create_wall_timer(
        std::chrono::milliseconds(30),
        [&timer_statistics]() {
            timer_statistics.record_now();
            static int count = 0;
            if(count++ > 200)
            {
                auto durations = timer_statistics.to_durations();
                // export to csv
                std::ofstream file("normal_timer.csv", std::ios::out);
                for(auto duration : durations)
                {
                    file << duration.count() << std::endl;
                    std::cout << duration.count() << std::endl;
                }

                // calculate statistics
                double sum = 0;
                for(auto duration : durations)
                {
                    sum += duration.count();
                }
                double mean = sum / durations.size();
                double variance = 0;
                for(auto duration : durations)
                {
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
        }
    );

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    return 0;
}
