#include <memory>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <cmath>
#include <deque>
#include <chrono>

#include <nlohmann/json.hpp>
#include "pubsub_srvcli/client_custom_msg.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

CustomClient::CustomClient()
: node_(rclcpp::Node::make_shared("custom_client")),
  client_(node_->create_client<pubsub_srvcli::srv::VectorDistance>("calculate_distance")),
  request_(std::make_shared<pubsub_srvcli::srv::VectorDistance::Request>()),
  i_(0)
{
}

bool CustomClient::load_json(const std::string &path, json &j)
{
    std::ifstream data(path);
    if (!data.is_open()) {
        std::cout << "veri.json dosyası bulunamadı." << std::endl;
        return false;
    }
    data >> j;
    return true;
}

int CustomClient::process_json_array(const json &j)
{
    // Servisin hazır olmasını bekle (orijinal mantık)
    while (!client_->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Data is not recieved.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try again later.");
    }

    for (const auto &item : j) {
        positions_.push_front(
            {item["x"].get<double>(), item["y"].get<double>(), item["z"].get<double>()});

        // Orijinalindeki gibi hep positions[i_] (i_ = 0) kullanıyoruz
        request_->x = positions_[i_].x;
        request_->y = positions_[i_].y;
        request_->z = positions_[i_].z;

        auto result_future = client_->async_send_request(request_);
        if (rclcpp::spin_until_future_complete(node_, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Distance: %f", result_future.get()->distance);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not found: VectorDistance");
        }
    }

    return 0;
}

int CustomClient::interactive_loop()
{
    char c = 'q';
    char q = 'a';

    // Kullanıcı etkileşimi (orijinal mantık korunarak, küçük bir iyileştirme:
    // q girilirse sayı istemeden çık)
    while (c != q) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "\n devam etmek için bir tuşa bas.\n çıkmak için q ya basın.");

        std::cin >> q;
        if (q == 'q')
            break;

        double x_, y_, z_;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3 adet double değeri gir");
        std::cin >> x_ >> y_ >> z_;

        positions_.push_front({x_, y_, z_});

        request_->x = positions_[i_].x;
        request_->y = positions_[i_].y;
        request_->z = positions_[i_].z;

        auto result_future = client_->async_send_request(request_);
        if (rclcpp::spin_until_future_complete(node_, result_future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                        "Distance: %f", result_future.get()->distance);
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not found: VectorDistance");
        }
    }

    return 0;
}

int CustomClient::run()
{
    // Orijinaldeki sabit yolu korudum
    const std::string default_path =
        "/home/revengeofthesob/ros3_ws/src/pubsub_srvcli/src/veri.json";

    return run(default_path);
}

int CustomClient::run(const std::string &json_path)
{
    json j;
    if (!load_json(json_path, j)) {
        return 1;
    }

    int ret = process_json_array(j);
    if (ret != 0) return ret;

    return interactive_loop();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    CustomClient app;
    int ret = app.run(); // JSON yolu istersen buraya parametreyle gönderebilirsin

    rclcpp::shutdown();
    return ret;
}
