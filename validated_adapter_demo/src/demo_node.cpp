// Demo node for validated type adapter
//
// This node demonstrates the use of validated type adapters
// generated from IDL annotations.

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// Include the standard ROS message
#include "sample_interfaces/msg/sensor_data.hpp"

// Include the validated type adapter
#include "sample_interfaces/msg/validated/sensor_data__validated.hpp"

using namespace std::chrono_literals;

// Use the type adapter
using SensorDataValidated = sample_interfaces::msg::validated::SensorDataValidated;
using SensorDataAdapter = rclcpp::TypeAdapter<SensorDataValidated, sample_interfaces::msg::SensorData>;

class ValidatedPublisherNode : public rclcpp::Node
{
public:
  ValidatedPublisherNode()
  : Node("validated_publisher")
  {
    // Create publisher using the type adapter
    publisher_ = this->create_publisher<SensorDataAdapter>("sensor_data", 10);

    // Timer to publish messages
    timer_ = this->create_wall_timer(
      1s, std::bind(&ValidatedPublisherNode::publish_message, this));

    RCLCPP_INFO(this->get_logger(), "Validated Publisher Node started");
    RCLCPP_INFO(this->get_logger(), "Publishing valid messages every 1 second...");
    RCLCPP_INFO(this->get_logger(), "After 5 valid messages, will attempt to publish invalid data");
  }

private:
  void publish_message()
  {
    SensorDataValidated msg;

    if (count_ < 5) {
      // Publish valid data
      msg.temperature = 25.0 + count_ * 5.0;  // 25, 30, 35, 40, 45 (all within -40 to 85)
      msg.humidity = 50.0 + count_ * 5.0;     // 50, 55, 60, 65, 70 (all within 0 to 100)
      msg.battery_voltage = 3.7;               // Within 3.0 to 4.2
      msg.sensor_id = count_ + 1;              // 1, 2, 3, 4, 5 (all >= 1)
      msg.status_code = 0;                     // Within 0 to 255
      msg.timestamp_ns = this->now().nanoseconds();
      msg.description = "Valid sensor reading #" + std::to_string(count_ + 1);

      try {
        publisher_->publish(msg);
        RCLCPP_INFO(
          this->get_logger(),
          "Published valid message: temp=%.1f, humidity=%.1f, voltage=%.1f, sensor_id=%u",
          msg.temperature, msg.humidity, msg.battery_voltage, msg.sensor_id);
      } catch (const sample_interfaces::msg::validated::ValidationError & e) {
        RCLCPP_ERROR(this->get_logger(), "Unexpected validation error: %s", e.what());
      }
    } else if (count_ == 5) {
      // Attempt to publish invalid data (temperature out of range)
      msg.temperature = 100.0;  // INVALID: exceeds max of 85
      msg.humidity = 50.0;
      msg.battery_voltage = 3.7;
      msg.sensor_id = 1;
      msg.status_code = 0;
      msg.timestamp_ns = this->now().nanoseconds();
      msg.description = "Invalid temperature test";

      RCLCPP_WARN(this->get_logger(), "Attempting to publish INVALID temperature (100.0 > max 85.0)...");

      try {
        publisher_->publish(msg);
        RCLCPP_ERROR(this->get_logger(), "ERROR: Invalid message was published without validation!");
      } catch (const sample_interfaces::msg::validated::ValidationError & e) {
        RCLCPP_INFO(this->get_logger(), "Validation caught invalid data: %s", e.what());
      }
    } else if (count_ == 6) {
      // Attempt to publish invalid data (humidity out of range)
      msg.temperature = 25.0;
      msg.humidity = -10.0;  // INVALID: below min of 0
      msg.battery_voltage = 3.7;
      msg.sensor_id = 1;
      msg.status_code = 0;
      msg.timestamp_ns = this->now().nanoseconds();
      msg.description = "Invalid humidity test";

      RCLCPP_WARN(this->get_logger(), "Attempting to publish INVALID humidity (-10.0 < min 0.0)...");

      try {
        publisher_->publish(msg);
        RCLCPP_ERROR(this->get_logger(), "ERROR: Invalid message was published without validation!");
      } catch (const sample_interfaces::msg::validated::ValidationError & e) {
        RCLCPP_INFO(this->get_logger(), "Validation caught invalid data: %s", e.what());
      }
    } else if (count_ == 7) {
      // Attempt to publish invalid data (sensor_id = 0)
      msg.temperature = 25.0;
      msg.humidity = 50.0;
      msg.battery_voltage = 3.7;
      msg.sensor_id = 0;  // INVALID: below min of 1
      msg.status_code = 0;
      msg.timestamp_ns = this->now().nanoseconds();
      msg.description = "Invalid sensor_id test";

      RCLCPP_WARN(this->get_logger(), "Attempting to publish INVALID sensor_id (0 < min 1)...");

      try {
        publisher_->publish(msg);
        RCLCPP_ERROR(this->get_logger(), "ERROR: Invalid message was published without validation!");
      } catch (const sample_interfaces::msg::validated::ValidationError & e) {
        RCLCPP_INFO(this->get_logger(), "Validation caught invalid data: %s", e.what());
      }
    } else if (count_ == 8) {
      // Demo: using is_valid() for non-throwing check
      msg.temperature = 200.0;  // INVALID
      msg.humidity = 50.0;
      msg.battery_voltage = 3.7;
      msg.sensor_id = 1;
      msg.status_code = 0;

      RCLCPP_INFO(this->get_logger(), "Demo: Using is_valid() for non-throwing validation check");
      if (msg.is_valid()) {
        RCLCPP_INFO(this->get_logger(), "  Message is valid");
      } else {
        RCLCPP_INFO(this->get_logger(), "  Message is INVALID (temperature=200.0)");
      }

      // Fix and check again
      msg.temperature = 25.0;
      if (msg.is_valid()) {
        RCLCPP_INFO(this->get_logger(), "  After fix: Message is valid (temperature=25.0)");
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "  Published fixed message");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Demo complete. Shutting down...");
      rclcpp::shutdown();
      return;
    }

    count_++;
  }

  rclcpp::Publisher<SensorDataAdapter>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ValidatedPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
