#ifndef CPP_PUBSUB_YILMAZ_SUBSCRIBER_HPP_
#define CPP_PUBSUB_YILMAZ_SUBSCRIBER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class yilmaz : public rclcpp::Node
{

    public:
        yilmaz();

    private:
         rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

#endif