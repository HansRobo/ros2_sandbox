//
// Created by hans on 24/10/27.
//

#ifndef REALTIME_BUFFER_HPP
#define REALTIME_BUFFER_HPP

#include <vector>
#include <iostream>
#include <chrono>


// enum class DefaultSeries{
//   DEFAULT,
// };
//
// template <typename Series = DefaultSeries
class TimerStatistics {
  public:
    using clock_type = std::chrono::high_resolution_clock;
    explicit TimerStatistics(int buffer_reserve_size = 1000) {
      records.reserve(buffer_reserve_size);
    }

    void record_now() {
      records.push_back(clock_type::now());
    }

    [[nodiscard]] auto to_durations() const
    {
      std::vector<std::chrono::duration<double>> durations;
      durations.reserve(records.size());
      for (size_t i = 1; i < records.size(); i++) {
        durations.emplace_back(records[i] - records[i - 1]);
      }
      return durations;
    }

    std::vector<clock_type::time_point> records;
};
#endif //REALTIME_BUFFER_HPP
