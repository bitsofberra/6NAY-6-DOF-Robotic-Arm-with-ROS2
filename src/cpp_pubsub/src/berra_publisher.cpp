#include "cpp_pubsub/berra_publisher.hpp"
#include <chrono>

using namespace std::chrono_literals;

BerraPublisher::BerraPublisher()
: Node("berra_publisher"), count_(0)
{
    publisher_ = this->create_publisher<std_msgs::msg::String>("berra_topic", 10);

    timer_ = this->create_wall_timer(
        800ms, std::bind(&BerraPublisher::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Berra Publisher has been started YEYYY!");
}

void BerraPublisher::timerCallback()
{
    std_msgs::msg::String msg;
    msg.data = "Drums Please, Yılmazzz! [" + std::to_string(count_++) + "]";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
    publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BerraPublisher>());
    rclcpp::shutdown();
    return 0;
}
