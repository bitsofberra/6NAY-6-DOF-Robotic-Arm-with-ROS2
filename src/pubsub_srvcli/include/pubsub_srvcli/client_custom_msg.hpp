#ifndef CLIENT_CUSTOM_MSG_HPP
#define CLIENT_CUSTOM_MSG_HPP

#include <deque>
#include <string>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp"

// Orijinalindekiyle aynı isimde basit vektör yapısı
struct Vec {
    double x, y, z;
};

class CustomClient {
public:
    CustomClient();
    // Varsayılan JSON yolu ile çalıştır
    int run();

    // İstersen JSON yolunu dışarıdan da verebilirsin
    int run(const std::string &json_path);

private:
    bool load_json(const std::string &path, nlohmann::json &j);
    int process_json_array(const nlohmann::json &j);
    int interactive_loop();

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<pubsub_srvcli::srv::VectorDistance>::SharedPtr client_;
    pubsub_srvcli::srv::VectorDistance::Request::SharedPtr request_;

    std::deque<Vec> positions_;
    int i_; // Orijinal mantığa sadık kalarak hep 0. elemanı kullanıyoruz
};

#endif // PUBSUB_SRVCLI_CLIENT_CUSTOM_MSG_HPP
