#ifndef BERRA_PUBLISHER_HPP_
#define BERRA_PUBLISHER_HPP_

#include <chrono>
#include <memory>
#include <string>

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

#endif  
