#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp"
#include <memory>
#include <cmath>

using VectorDistance = pubsub_srvcli::srv::VectorDistance;

void handle_request(
    const std::shared_ptr<VectorDistance::Request> request,
    std::shared_ptr<VectorDistance::Response> response)
{
    response->distance = std::sqrt(
        request->x * request->x +
        request->y * request->y +
        request->z * request->z);

    RCLCPP_INFO(
      rclcpp::get_logger("server_custom_msg"),
      "Request: x=%.2f, y=%.2f, z=%.2f",
      request->x, request->y, request->z);

    RCLCPP_INFO(
      rclcpp::get_logger("server_custom_msg"),
      "Response: distance=%.2f",
      response->distance);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("server_custom_msg");

    auto service = node->create_service<VectorDistance>(
        "calculate_distance", &handle_request);

    RCLCPP_INFO(
      rclcpp::get_logger("server_custom_msg"),
      "Service ready: calculate_distance");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
