#include "pubsub_srvcli/publisher_custom_msg.hpp"
#include <chrono>

using namespace std::chrono_literals;

CustomPublisher::CustomPublisher()
: Node("custom_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);

    timer_ = this->create_wall_timer(
        800ms, std::bind(&CustomPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Publisher has been started send the data.");
}

void CustomPublisher::timerCallback()
{
    std_msgs::msg::String msg;
    msg.data = "Drums Please, World! [" + std::to_string(count_++) + "]";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomPublisher>());
    rclcpp::shutdown();
    return 0;
}
