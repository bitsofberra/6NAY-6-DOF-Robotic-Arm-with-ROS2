#ifndef PUBLISHER_CUSTOM_MSG_HPP_
#define PUBLISHER_CUSTOM_MSG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class CustomPublisher : public rclcpp::Node
{
public:
  CustomPublisher();

private:
  void timerCallback();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

#endif  