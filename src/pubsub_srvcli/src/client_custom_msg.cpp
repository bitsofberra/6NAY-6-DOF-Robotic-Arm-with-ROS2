#include <memory>
#include <cstdlib> 
#include <fstream>
#include <cmath>
#include <deque>
#include "pubsub_srvcli/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pubsub_srvcli/srv/vector_distance.hpp" 


using namespace std::chrono_literals;
using json = nlohmann::json;


struct Vec {
    double x, y, z;
};

int main()
{
   std::ifstream data("/home/yilmaz/ros2_ws/src/pubsub_srvcli/src/veri.json");

    if (!data.is_open()) {
        std::cout<< "veri.json dosyası bulunamadı." << std::endl;
        return 1;
    }
    json j;
    data >> j;

    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);

    std::deque<Vec> positions;

    auto node = rclcpp::Node::make_shared("custom_client");

    auto client = node->create_client<pubsub_srvcli::srv::VectorDistance>("VectorDistance");

    auto request = std::make_shared<pubsub_srvcli::srv::VectorDistance::Request>();

    char c = 'q' ;
    char q;

    for (const auto& item : j)
    {
      positions.push_front({item["x"].get<double>(), item["y"].get<double>(), item["z"].get<double>()});  
        
        request->x = (positions[0].x);
        request->y = (positions[0].y);
        request->z = (positions[0].z);

    }
    
    while (!client->wait_for_service(3s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Data is not recieved.");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Try again later.");
    }

    int i=0;

    while (c!=q)
    {
    
    request->x = (positions[i].x);
    request->y = (positions[i].y);
    request->z = (positions[i].z);

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Distance: %f", result_future.get()->distance);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Not found: VectorDistance");
    }

    double x_, y_, z_ ;    

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3 adet double değeri gir");

    std::cin >> x_ >> y_ >> z_ ;

    positions.push_front({x_, y_, z_});

    if(i != 0){

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\n devam etmek için bir tuşa bas.\n çıkmak için q ya basın.");

    std::cin >> q ;
    }

    i+=1;     

}

    rclcpp::shutdown();
    return 0;
}
