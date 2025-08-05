#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class yilmaz : public rclcpp::Node
{


public:

    yilmaz()
    :Node("andromeda")

    {
        auto topic_callback =

        [this](std_msgs::msg::String::UniquePtr msg) -> void {


            RCLCPP_INFO(this->get_logger(), "duydum ki sefere çıkmayı kuruyormuşsun : '%s'", msg->data.c_str());


        };

        subscription_ =
            this->create_subscription<std_msgs::msg::String>("berra_topic", 40, topic_callback);

    };

    
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{

rclcpp::init(argc, argv);
rclcpp::spin(std::make_shared<yilmaz>());
rclcpp::shutdown();
return 0;
}