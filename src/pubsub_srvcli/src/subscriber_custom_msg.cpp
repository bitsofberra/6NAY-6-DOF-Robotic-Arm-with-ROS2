#include <memory>
#include "pubsub_srvcli/subscriber_custom_msg.hpp"

CustomSubscriber::CustomSubscriber()
: Node("custom_subscriber")
{
    auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard you: '%s'", msg->data.c_str());
    };

    subscription_ = this->create_subscription<std_msgs::msg::String>("string_topic", 10, topic_callback);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CustomSubscriber>());
    rclcpp::shutdown();
    return 0;
}
