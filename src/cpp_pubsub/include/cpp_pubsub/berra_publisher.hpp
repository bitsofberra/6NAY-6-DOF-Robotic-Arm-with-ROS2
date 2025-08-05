#ifndef CPP_PUBSUB_BERRA_PUBLISHER_HPP_
#define CPP_PUBSUB_BERRA_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class BerraPublisher : public rclcpp::Node
{
public:
    BerraPublisher();

private:
    void timerCallback();

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#endif  // CPP_PUBSUB_BERRA_PUBLISHER_HPP_
