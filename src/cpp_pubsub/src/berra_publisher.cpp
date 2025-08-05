#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class BerraPublisher : public rclcpp::Node
{
public:
    BerraPublisher() : Node("berra_publisher"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("berra_topic", 10);

        timer_ = this->create_wall_timer(
            800ms, std::bind(&BerraPublisher::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "Berra Publisher has been started YEYYY!");
    }

private:
    void timerCallback()
    {
        std_msgs::msg::String msg;
        msg.data = "Drums Please, Yılmazzz! [" + std::to_string(count_++) + "]";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BerraPublisher>());
    rclcpp::shutdown();
    return 0;
}
