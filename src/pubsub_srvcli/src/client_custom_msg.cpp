#include <memory>
#include <cstdlib> 
#include <fstream>
#include <cmath>
#include "pubsub_srvcli/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp" 


using namespace std::chrono_literals;
using json = nlohmann::json;

int main()
{

   std::ifstream data("/home/yilmaz/ros2_ws/src/pubsub_srvcli/src/veri.json");

    if (!data.is_open()) {
        std::cerr << "veri.json dosyası bulunamadı." << std::endl;
        return 1;
    }
    json j;
    data >> j;



    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);

    
    auto node = rclcpp::Node::make_shared("custom_client");

    auto client = node->create_client<pubsub_srvcli::srv::VectorDistance>("VectorDistance");

    
    auto request = std::make_shared<pubsub_srvcli::srv::VectorDistance::Request>();

   


    request->x = (j["x"]);
    request->y = (j["y"]);
    request->z = (j["z"]);

    while (!client->wait_for_service(5s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Data is not recieved.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try again later.");
    }

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f", result_future.get()->distance);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not found: VectorDistance");
    }

    rclcpp::shutdown();
    return 0;
}
