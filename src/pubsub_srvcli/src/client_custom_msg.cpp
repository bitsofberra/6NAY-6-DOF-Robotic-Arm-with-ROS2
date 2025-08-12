
#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>

using json = nlohmann::json;
using namespace std::chrono_literals;

int main(int , char** )
{
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("custom_client");
  auto client = node->create_client<pubsub_srvcli::srv::VectorDistance>("calculate_distance");

  std::ifstream file("src/pubsub_srvcli/src/data.json");
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "data.json dosyası açılamadı!");
    return 1;
  }

  json j;
  file >> j;
  file.close();

  auto request = std::make_shared<pubsub_srvcli::srv::VectorDistance::Request>();
  request->x = j.at("x").get<double>();
  request->y = j.at("y").get<double>();
  request->z = j.at("z").get<double>();

  if (!client->wait_for_service(5s)) {
    RCLCPP_ERROR(node->get_logger(), "Servis bulunamadı: calculate_distance");
    return 1;
  }

  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    double d = result_future.get()->distance;
    RCLCPP_INFO(node->get_logger(), "Distance: %.2f", d);
  } else {
    RCLCPP_ERROR(node->get_logger(), "Servis çağrısı başarısız");
  }

  rclcpp::shutdown();
  return 0;
}
