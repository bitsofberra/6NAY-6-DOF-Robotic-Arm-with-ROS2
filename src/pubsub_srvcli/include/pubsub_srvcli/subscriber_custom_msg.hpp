#ifndef SUBSCRIBER_CUSTOM_MSG_HPP_
#define SUBSCRIBER_CUSTOM_MSG_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class CustomSubscriber : public rclcpp::Node
{
public:
  CustomSubscriber();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  