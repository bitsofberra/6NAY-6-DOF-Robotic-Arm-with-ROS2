#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp"  

#include <memory>
#include <cstdlib>  

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Please enter 3 float value.");
        return 1;  }

    auto node = rclcpp::Node::make_shared("custom_client");

    auto client = node->create_client<pubsub_srvcli::srv::VectorDistance>("VectorDistance");

    
    auto request = std::make_shared<pubsub_srvcli::srv::VectorDistance::Request>();

    request->x = std::atof(argv[1]);
    request->y = std::atof(argv[2]);
    request->z = std::atof(argv[3]);

    while (!client->wait_for_service(5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Data is not recieved.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try again later.");
    }

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f", result_future.get()->distance);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not found: VectorDistance");
    }

    rclcpp::shutdown();
    return 0;
}
