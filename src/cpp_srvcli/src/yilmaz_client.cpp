#include "rclcpp/rclcpp.hpp"
#include "cpp_srvcli/srv/server.hpp"  // Servis dosyasının doğru yolu

#include <memory>
#include <cstdlib>  // atoll için

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc != 4) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "3 adet float değeri girin");
        return 1;  // return1 → return 1;
    }

    auto node = rclcpp::Node::make_shared("yilmaz");

    // Servis tipi büyük harf ile başlar: Server
    auto client = node->create_client<cpp_srvcli::srv::Server>("yilmaz_server");

    // İstek oluştur
    auto request = std::make_shared<cpp_srvcli::srv::Server::Request>();

    // argv stringlerini float'a çevirmek için atof kullanılır, atoll int64 döner. 
    request->x = std::atof(argv[1]);
    request->y = std::atof(argv[2]);
    request->z = std::atof(argv[3]);

    // Servis hazır olana kadar bekle
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Veri gelmedi, program kapanıyor");
            return 1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Daha sonra dene");
    }

    // İstek gönder
    auto result_future = client->async_send_request(request);

    // Sonuç için bekle
    if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS) {
        // result_future.get() ile cevabı al
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Uzunluk: %f", result_future.get()->distance);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_three_ints");
    }

    rclcpp::shutdown();
    return 0;
}
