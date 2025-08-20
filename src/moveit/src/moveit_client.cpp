#include <memory>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <deque>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "moveit/srv/vector_distance.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

struct Vec { double x, y, z; };

int main(int argc, char** argv)
{
  const char* json_path = "/home/yilmaz/ros2_ws/src/moveit/veri.json";

  // JSON'u aç ve diziye yükle
  std::ifstream data(json_path);
  if (!data.is_open()) {
    std::cout << "veri.json dosyası bulunamadı." << std::endl;
    return 1;
  }
  json j;
  try {
    data >> j;
    if (!j.is_array()) j = json::array();
  } catch (...) {
    std::cout << "veri.json okunamadı ya da geçersiz. Boş dizi ile devam ediliyor." << std::endl;
    j = json::array();
  }

  rclcpp::init(argc, argv);
  auto node   = rclcpp::Node::make_shared("custom_client");
  auto client = node->create_client<moveit::srv::VectorDistance>("/VectorDistance");

  while (!client->wait_for_service(3s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "ROS kapatıldı / servis beklenemiyor.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "VectorDistance bekleniyor...");
  }

  std::deque<Vec> positions;
  auto request = std::make_shared<moveit::srv::VectorDistance::Request>();
  int i = 0;

  // 1) JSON'daki kayıtları deque'e doldur ve SUNUCUYA GÖNDER
  try {
    for (const auto& item : j) {
      double x = item.value("x", 0.0);
      double y = item.value("y", 0.0);
      double z = item.value("z", 0.0);

      positions.push_back({ x, y, z });

      request->x = positions[i].x;
      request->y = positions[i].y;
      request->z = positions[i].z;

      auto result_future = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Distance: %s", result_future.get()->distance.c_str());
        // Eğer distance float/double ise:
        // RCLCPP_INFO(node->get_logger(), "Distance: %.3f", result_future.get()->distance);
      } else {
        RCLCPP_ERROR(node->get_logger(), "Not found: VectorDistance");
      }
      ++i;
      rclcpp::sleep_for(100ms);
    }
  } catch (...) {
    RCLCPP_WARN(node->get_logger(), "veri.json içeriği beklenmedik formatta olabilir; bazı kayıtlar atlandı.");
  }

  // 2) KONSOL döngüsü yerine: GUI'den gelenleri dinle
  auto sub = node->create_subscription<geometry_msgs::msg::Point>(
    "/vector_distance_input", 10,
    [&, node, client, json_path](const geometry_msgs::msg::Point::SharedPtr msg)
    {
      // a) deque'e ekle
      positions.push_back({ msg->x, msg->y, msg->z });

      // b) JSON'a append edip dosyayı güncelle
      try {
        j.push_back({ {"x", msg->x}, {"y", msg->y}, {"z", msg->z} });
        std::ofstream out(json_path, std::ios::trunc);
        out << j.dump(2);
        RCLCPP_INFO(node->get_logger(), "veri.json guncellendi (toplam %zu kayit).", j.size());
      } catch (...) {
        RCLCPP_WARN(node->get_logger(), "veri.json'a yazma basarisiz.");
      }

      // c) Sunucuya gönder (callback ile sonucu logla)
      auto req = std::make_shared<moveit::srv::VectorDistance::Request>();
      req->x = positions[i].x;
      req->y = positions[i].y;
      req->z = positions[i].z;

      client->async_send_request(
        req,
        [node](rclcpp::Client<moveit::srv::VectorDistance>::SharedFuture resp){
          RCLCPP_INFO(node->get_logger(), "Distance: %s", resp.get()->distance.c_str());
          // Eğer distance float/double ise:
          // RCLCPP_INFO(node->get_logger(), "Distance: %.3f", resp.get()->distance);
        }
      );
      ++i;
    }
  );

  RCLCPP_INFO(node->get_logger(), "custom_client calisiyor. GUI'den /vector_distance_input bekleniyor.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
