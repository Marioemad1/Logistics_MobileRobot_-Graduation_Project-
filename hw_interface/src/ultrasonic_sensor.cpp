#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include <lgpio.h>
#include <chrono>
#include <thread>
#include <limits>

class UltrasonicNode : public rclcpp::Node
{
public:
  UltrasonicNode() : Node("ultrasonic_sensor")
  {
    this->declare_parameter("trig_pin", 17);
    this->declare_parameter("echo_pin", 13);         
    this->declare_parameter("frame_id", "ultra1_link");

    trig_pin_ = this->get_parameter("trig_pin").as_int();
    echo_pin_ = this->get_parameter("echo_pin").as_int();
    frame_id_ = this->get_parameter("frame_id").as_string();

    h_gpio_ = lgGpiochipOpen(0);
    if (h_gpio_ < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open GPIO chip: %s", lguErrorText(h_gpio_));
      throw std::runtime_error("GPIO open failed");
    }

    // Claim pins
    if (lgGpioClaimOutput(h_gpio_, 0, trig_pin_, 0) < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to claim trig pin %d", trig_pin_);
      lgGpiochipClose(h_gpio_);
      throw std::runtime_error("Trig claim failed");
    }

    if (lgGpioClaimInput(h_gpio_, 0, echo_pin_) < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to claim echo pin %d", echo_pin_);
      lgGpiochipClose(h_gpio_);
      throw std::runtime_error("Echo claim failed");
    }

    publisher_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasonic/range", 10);

    // 10 Hz = 100 ms — safe starting point for HC-SR04
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&UltrasonicNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),"Pins is Clamid...");
    RCLCPP_INFO(this->get_logger(), "Ultrasonic node started (trig=%d, echo=%d)", trig_pin_, echo_pin_);
  }

  ~UltrasonicNode()
  {
    if (h_gpio_ >= 0)
    {
      lgGpioFree(h_gpio_, trig_pin_);
      lgGpioFree(h_gpio_, echo_pin_);
      lgGpiochipClose(h_gpio_);
    }
  }

private:
  float get_distance()
  {
    lgGpioWrite(h_gpio_, trig_pin_, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    lgGpioWrite(h_gpio_, trig_pin_, 0);

    auto timeout_start = std::chrono::steady_clock::now();

    // Wait for echo HIGH
    while (lgGpioRead(h_gpio_, echo_pin_) == 0)
    {
      if (std::chrono::steady_clock::now() - timeout_start > std::chrono::milliseconds(30))
      {
        RCLCPP_WARN(this->get_logger(), "No echo start");
        return std::numeric_limits<float>::infinity();
      }
    }

    auto pulse_start = std::chrono::steady_clock::now();
    timeout_start = std::chrono::steady_clock::now();

    // Wait for echo LOW
    while (lgGpioRead(h_gpio_, echo_pin_) == 1)
    {
      if (std::chrono::steady_clock::now() - timeout_start > std::chrono::milliseconds(30))
      {
        RCLCPP_WARN(this->get_logger(), "Echo pulse too long");
        return std::numeric_limits<float>::infinity();
      }
    }

    auto pulse_end = std::chrono::steady_clock::now();

    auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
                         pulse_end - pulse_start).count();

    float distance_cm = (static_cast<float>(duration_us) * 0.0343f) / 2.0f;

    // Basic sanity check
    if (distance_cm < 2.0f || distance_cm > 400.0f)
    {
      return std::numeric_limits<float>::infinity();
    }

    return distance_cm;
  }

  void timer_callback()
  {
    auto msg = sensor_msgs::msg::Range();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    msg.field_of_view = 0.3f;
    msg.min_range = 0.02f;
    msg.max_range = 4.0f;
    msg.range = get_distance() / 100.0f;   // cm → meters (ROS standard)

    publisher_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int h_gpio_ = -1;
  int trig_pin_ = 0;
  int echo_pin_ = 0;
  std::string frame_id_;
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<UltrasonicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}
