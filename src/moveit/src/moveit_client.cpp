#include <memory>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <deque>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "moveit/srv/vector_distance.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

struct Vec { double x, y, z; };

int main(int argc, char** argv)
{
  std::ifstream data("/home/yilmaz/ros2_ws/src/moveit/veri.json");
  if (!data.is_open()) {
    std::cout << "veri.json dosyası bulunamadı." << std::endl;
    return 1;
  }
  json j;
  data >> j;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("custom_client");
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

  for (const auto& item : j) {
    positions.push_back({ item["x"].get<double>(), item["y"].get<double>(), item["z"].get<double>() });

    request->x = positions[i].x;
    request->y = positions[i].y;
    request->z = positions[i].z;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Distance: %s", result_future.get()->distance.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Not found: VectorDistance");
    }
    ++i;

    rclcpp::sleep_for(100ms);
  }

  char c = 'a';
  while (c != 'q') {
    RCLCPP_INFO(node->get_logger(), "\nDevam etmek için bir tuşa basın.\nÇıkmak için 'q' ya basın.");
    std::cin >> c;
    if (c == 'q') break;

    double x_, y_, z_;
    RCLCPP_INFO(node->get_logger(), "3 adet double değeri giriniz (x y z):");
    std::cin >> x_ >> y_ >> z_;

    positions.push_back({ x_, y_, z_ });

    request->x = positions[i].x;
    request->y = positions[i].y;
    request->z = positions[i].z;

    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node->get_logger(), "Distance: %s", result_future.get()->distance.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Not found: VectorDistance");
    }
    ++i;
  }

  rclcpp::shutdown();
  return 0;
}
