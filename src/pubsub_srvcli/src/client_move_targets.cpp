#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <fstream>
#include <iostream>
#include <deque>
#include <chrono>

#include <nlohmann/json.hpp>
#include "pubsub_srvcli/srv/vector_distance.hpp"
#include <std_srvs/srv/trigger.hpp>

using json = nlohmann::json;
using VectorDistance = pubsub_srvcli::srv::VectorDistance;

struct Target { double x, y, z; };

static bool load_json(const std::string &path, json &j)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    std::cout << "veri.json bulunamadı: " << path << std::endl;
    return false;
  }
  f >> j;
  return true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("move_client");

  // Argümandan gelmezse varsayılan JSON yolu
  std::string json_path = (argc > 1) ? argv[1]
    : "/home/revengeofthesob/ros3_ws/src/pubsub_srvcli/src/veri.json";

  json j;
  if (!load_json(json_path, j)) {
    rclcpp::shutdown();
    return 1;
  }

  // İlk 3 hedefi kuyruğa at
  std::deque<Target> q;
  size_t n = std::min<size_t>(3, j.size());
  for (size_t k = 0; k < n; ++k) {
    const auto &item = j[k];
    q.push_back(Target{
      item.at("x").get<double>(),
      item.at("y").get<double>(),
      item.at("z").get<double>()
    });
  }
  if (q.empty()) {
    std::cout << "JSON boş ya da 3D hedef yok." << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  auto client = node->create_client<VectorDistance>("calculate_distance");
  while (!client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "ROS kapandı, servis beklenemedi.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service bekleniyor: calculate_distance");
  }

  // Hedefleri sırayla gönder
  while (!q.empty()) {
    auto t = q.front(); q.pop_front();

    auto req = std::make_shared<VectorDistance::Request>();
    req->x = t.x; req->y = t.y; req->z = t.z;

    RCLCPP_INFO(node->get_logger(),
                "Request -> x=%.3f y=%.3f z=%.3f (kalan=%zu)", t.x, t.y, t.z, q.size());

    auto fut = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, fut) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto res = fut.get();
      RCLCPP_INFO(node->get_logger(), "Response distance=%.3f", res->distance);
    } else {
      RCLCPP_ERROR(node->get_logger(), "Service çağrısı başarısız.");
      rclcpp::shutdown();
      return 1;
    }
  }

  // Bitince home’a dön
  auto home_client = node->create_client<std_srvs::srv::Trigger>("go_home");
  while (!home_client->wait_for_service(std::chrono::seconds(2))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "ROS kapandı, go_home beklenemedi.");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service bekleniyor: go_home");
  }

  auto home_req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto home_fut = home_client->async_send_request(home_req);
  if (rclcpp::spin_until_future_complete(node, home_fut) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    auto res = home_fut.get();
    if (res->success) {
      RCLCPP_INFO(node->get_logger(), "Home OK: %s", res->message.c_str());
    } else {
      RCLCPP_WARN(node->get_logger(), "Home FAIL: %s", res->message.c_str());
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "go_home çağrısı başarısız.");
  }

  RCLCPP_INFO(node->get_logger(), "Tüm hedefler işlendi.");
  rclcpp::shutdown();
  return 0;
}
