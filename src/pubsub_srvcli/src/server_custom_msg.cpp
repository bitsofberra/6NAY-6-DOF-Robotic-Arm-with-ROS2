#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp"  
#include <memory>
#include <cmath>  


void add(const std::shared_ptr<pubsub_srvcli::srv::VectorDistance::Request> request,
         std::shared_ptr<pubsub_srvcli::srv::VectorDistance::Response> response)
{
   
    response->distance = std::sqrt(request->x * request->x + request->y * request->y + request->z * request->z);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Request: x=%.2f, y=%.2f, z=%.2f", request->x, request->y, request->z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
                "Response: distance=%.2f", response->distance);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("server_custom_msg");

    rclcpp::Service<pubsub_srvcli::srv::VectorDistance>::SharedPtr service =
        node->create_service<pubsub_srvcli::srv::VectorDistance>("VectorDistance", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service ready: calculate_distance");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
