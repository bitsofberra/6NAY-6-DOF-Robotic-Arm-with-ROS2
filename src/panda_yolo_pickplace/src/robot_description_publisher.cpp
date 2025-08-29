#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <cstdio>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class RobotDescriptionPublisher : public rclcpp::Node {
public:
  RobotDescriptionPublisher()
  : rclcpp::Node("robot_description_publisher")
  {
    // Latched-benzeri QoS
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    pub_ = this->create_publisher<std_msgs::msg::String>("/panda/robot_description", qos);

    const std::string pkg = "panda_yolo_pickplace";
    const std::string share = ament_index_cpp::get_package_share_directory(pkg);
    const std::string xacro_file = share + "/urdf/panda_gz.urdf.xacro";
    const std::string controllers_yaml = share + "/config/controllers.yaml";

    // xacro çalıştır
    std::stringstream cmd;
    cmd << "xacro " << xacro_file << " controllers_yaml:=" << controllers_yaml;
    RCLCPP_INFO(get_logger(), "Running: %s", cmd.str().c_str());

    std::array<char, 4096> buffer{};
    std::string urdf_out;
    FILE* pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) throw std::runtime_error("xacro başlatılamadı (popen)");

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
      urdf_out += buffer.data();
    }
    int rc = pclose(pipe);
    if (rc != 0 || urdf_out.empty()) {
      RCLCPP_FATAL(get_logger(), "xacro çıktısı boş/hatali! rc=%d", rc);
      throw std::runtime_error("xacro failed");
    }

    msg_.data = urdf_out;

    // Anında yayınla ve periyodik tekrarla (geç gelen aboneler için)
    pub_->publish(msg_);
    RCLCPP_INFO(get_logger(), "Published robot_description to /panda/robot_description (latched & repeating)");
    timer_ = this->create_wall_timer(2s, [this](){ pub_->publish(msg_); });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std_msgs::msg::String msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDescriptionPublisher>());
  rclcpp::shutdown();
  return 0;
}
