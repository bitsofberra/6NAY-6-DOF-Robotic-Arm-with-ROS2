#ifndef CLIENT_CUSTOM_MSG_HPP
#define CLIENT_CUSTOM_MSG_HPP

#include <memory>
#include <string>
#include <deque>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp"

struct Vec {
    double x, y, z;
};

class CustomClient
{
public:
    CustomClient(const std::string &node_name,
                 const std::string &service_name,
                 const std::string &json_file);

    void run();
    
    private:
    void send_json_data();
    void input_loop();
    void send_request(double x, double y, double z);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<pubsub_srvcli::srv::VectorDistance>::SharedPtr client_;
    std::deque<Vec> positions_;
    std::string json_file_;
};

#endif
