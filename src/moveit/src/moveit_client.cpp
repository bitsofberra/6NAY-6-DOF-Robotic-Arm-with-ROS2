#include <memory>
#include <fstream>
#include <deque>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "moveit/srv/vector_distance.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("custom_client");
  auto client = node->create_client<moveit::srv::VectorDistance>("/VectorDistance");

  // Servis çağrısı helper'ı
  auto send_req = [node, client](double x, double y, double z)
  {
    if (!client->wait_for_service(0s)) {
      RCLCPP_WARN(node->get_logger(), "VectorDistance servisi hazir degil.");
      return;
    }
    auto req = std::make_shared<moveit::srv::VectorDistance::Request>();
    req->x = x; req->y = y; req->z = z;

    client->async_send_request(
      req,
      [node](rclcpp::Client<moveit::srv::VectorDistance>::SharedFuture resp) {
        RCLCPP_INFO(node->get_logger(), "Response: %s", resp.get()->distance.c_str());
      }
    );
  };

  // GUI'den gelen (x,y,z)
  auto sub = node->create_subscription<geometry_msgs::msg::Point>(
    "/vector_distance_input", 10,
    [send_req, node](geometry_msgs::msg::Point::ConstSharedPtr msg) {
      RCLCPP_INFO(node->get_logger(), "GUI -> x=%.3f y=%.3f z=%.3f", msg->x, msg->y, msg->z);
      send_req(msg->x, msg->y, msg->z);
    }
  );

  // (Opsiyonel) JSON’dan ön değerleri gönder
  try {
    std::ifstream data("/home/yilmaz/ros2_ws/src/moveit/veri.json");
    if (data.is_open()) {
      json j; data >> j;
      for (const auto& item : j) {
        double x = item.value("x", 0.0);
        double y = item.value("y", 0.0);
        double z = item.value("z", 0.0);
        RCLCPP_INFO(node->get_logger(), "JSON -> x=%.3f y=%.3f z=%.3f", x, y, z);
        send_req(x, y, z);
        rclcpp::sleep_for(100ms);
      }
    } else {
      RCLCPP_INFO(node->get_logger(), "veri.json yok, GUI'den bekleniyor...");
    }
  } catch (...) {
    RCLCPP_WARN(node->get_logger(), "veri.json okunamadı ya da gecersiz.");
  }

  RCLCPP_INFO(node->get_logger(), "custom_client calisiyor. GUI'den /vector_distance_input bekleniyor.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
