#include "rclcpp/rclcpp.hpp"
#include "cpp_srvcli/srv/vector_distance.hpp"

#include <memory>

void add(const std::shared_ptr<cpp_srvcli::srv::VectorDistance::Request> request,
            std::shared_ptr<cpp_srvcli::srv::VectorDistance::Response> response)
{
    response->distance = std::sqrt(request->x*request->x + request->y*request->y + request->z*request->z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Request: x=%.2f, y=%.2f, z=%.2f",
                    request->x, request->y, request->z);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: distance=%.2f", response->distance);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("berra_server");

    rclcpp::Service<cpp_srvcli::srv::VectorDistance>::SharedPtr service =
        node->create_service<cpp_srvcli::srv::VectorDistance>("VectorDistance", &add);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service ready: calculate_distance");

    rclcpp::spin(node);
    rclcpp::shutdown();
}