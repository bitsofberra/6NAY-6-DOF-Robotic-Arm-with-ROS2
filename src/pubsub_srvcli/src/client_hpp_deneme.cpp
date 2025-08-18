#include "pubsub_srvcli/client_custom_msg.hpp"
#include <fstream>
#include <iostream>

using json = nlohmann::json;
using namespace std::chrono_literals;

CustomClient::CustomClient(const std::string &node_name,
                           const std::string &service_name,
                           const std::string &json_file)
    : json_file_(json_file)
{
    node_ = rclcpp::Node::make_shared(node_name);
    client_ = node_->create_client<pubsub_srvcli::srv::VectorDistance>(service_name);
}

void CustomClient::send_json_data()
{
    std::ifstream data(json_file_);
    if (!data.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "JSON dosyası bulunamadı: %s", json_file_.c_str());
        return;
    }

    json j;
    data >> j;

    for (const auto &item : j)
    {
        positions_.push_front({item["x"].get<double>(), item["y"].get<double>(), item["z"].get<double>()});
        send_request(positions_.front().x, positions_.front().y, positions_.front().z);
    }
}

void CustomClient::send_request(double x, double y, double z)
{
    auto request = std::make_shared<pubsub_srvcli::srv::VectorDistance::Request>();
    request->x = x;
    request->y = y;
    request->z = z;

    while (!client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Service kullanılamıyor, çıkılıyor.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Servis bekleniyor...");
    }

    auto result_future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f", result_future.get()->distance);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Servisten cevap alınamadı.");
    }
}

void CustomClient::input_loop()
{
    char q = 0;
    while (q != 'q')
    {
        double x, y, z;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3 adet double değeri giriniz:");
        std::cin >> x >> y >> z;

        positions_.push_front({x, y, z});
        send_request(x, y, z);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Devam etmek için herhangi bir tuş, çıkmak için 'q' girin:");
        std::cin >> q;
    }
}

void CustomClient::run()
{
    send_json_data();
    input_loop();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<CustomClient>(
        "custom_client",
        "VectorDistance",
        "/home/yilmaz/ros2_ws/src/pubsub_srvcli/src/veri.json"
    );

    client_node->run();

    rclcpp::shutdown();
    return 0;
}

