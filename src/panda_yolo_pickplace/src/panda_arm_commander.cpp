#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class PandaArmCommander : public rclcpp::Node
{
public:
  PandaArmCommander()
  : rclcpp::Node("panda_arm_commander")
  {
    // --- Parametreleri ilan et (ilk açılışta defaultlarla oluşur) ---
    declare_if_needed<std::vector<double>>("home", std::vector<double>(7, 0.0));
    declare_if_needed<std::vector<double>>("pick", std::vector<double>(7, 0.0));
    declare_if_needed<std::vector<double>>("place", std::vector<double>(7, 0.0));
    declare_if_needed<std::string>("controller_mode", std::string("position")); // "position" | "trajectory"

    // --- Publisher'lar ---
    // Şu an aktif olan kontrolör: JointGroupPositionController
    position_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/panda/panda_arm_controller/commands", rclcpp::SystemDefaultsQoS());

    // İleride JointTrajectoryController kullanırsan diye hazır:
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/panda/panda_arm_controller/joint_trajectory", rclcpp::SystemDefaultsQoS());

    // --- Servis: /panda/execute_pick_place ---
    srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/panda/execute_pick_place",
      std::bind(&PandaArmCommander::on_trigger, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "PandaArmCommander hazır. Servisi çalıştır: ros2 service call /panda/execute_pick_place std_srvs/srv/Trigger {}");
  }

private:
  // Parametre okuma yardımcıları ------------------------------------------------
  template<typename T>
  void declare_if_needed(const std::string & name, const T & default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
  }

  std::vector<double> p_(const std::string & name)
  {
    // rclcpp::Parameter pointer dönmez; doğrudan değeri iste.
    std::vector<double> v;
    if (!this->has_parameter(name)) {
      // İlk kez isteniyorsa 7 elemanlı 0 vektörü ile ilan et
      this->declare_parameter<std::vector<double>>(name, std::vector<double>(7, 0.0));
    }
    if (!this->get_parameter(name, v)) {
      RCLCPP_WARN(get_logger(), "Parametre '%s' okunamadı; 7 elemanlı 0.0 vektörü kullanıyorum.", name.c_str());
      v.assign(7, 0.0);
    }
    if (v.size() != 7) {
      RCLCPP_WARN(get_logger(),
        "Parametre '%s' %zu elemanlı; 7 olması gerekir. Boyutu 7'ye çekip eksikleri 0 ile dolduruyorum.",
        name.c_str(), v.size());
      v.resize(7, 0.0);
    }
    return v;
  }

  // Komut göndericiler ----------------------------------------------------------
  void send_position_command(const std::vector<double> & q)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = q;
    position_pub_->publish(msg);
  }

  void send_trajectory_command(const std::vector<double> & q, double sec)
  {
    trajectory_msgs::msg::JointTrajectory traj;
    traj.joint_names = {
      "panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"
    };
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions = q;
    pt.time_from_start = rclcpp::Duration::from_seconds(sec);
    traj.points.push_back(pt);
    traj_pub_->publish(traj);
  }

  bool use_trajectory_mode() const
  {
    std::string mode = this->get_parameter("controller_mode").as_string();
    return (mode == "trajectory");
  }

  // Servis callback -------------------------------------------------------------
  void on_trigger(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // Parametreleri çek
    const auto q_home  = p_("home");
    const auto q_pick  = p_("pick");
    const auto q_place = p_("place");

    // Komut gönder
    try {
      if (use_trajectory_mode()) {
        send_trajectory_command(q_home,  2.0);
        rclcpp::sleep_for(2200ms);
        send_trajectory_command(q_pick,  2.0);
        rclcpp::sleep_for(2200ms);
        send_trajectory_command(q_place, 2.0);
        rclcpp::sleep_for(2200ms);
        send_trajectory_command(q_home,  2.0);
        rclcpp::sleep_for(2200ms);
      } else {
        // Position controller (Float64MultiArray)
        send_position_command(q_home);
        rclcpp::sleep_for(1500ms);
        send_position_command(q_pick);
        rclcpp::sleep_for(1500ms);
        send_position_command(q_place);
        rclcpp::sleep_for(1500ms);
        send_position_command(q_home);
        rclcpp::sleep_for(1500ms);
      }

      res->success = true;
      res->message = "Pick&Place komut dizisi gönderildi.";
      RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    } catch (const std::exception & e) {
      res->success = false;
      res->message = std::string("Hata: ") + e.what();
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
    }
  }

  // Üyeler ----------------------------------------------------------------------
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
};

// main --------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PandaArmCommander>());
  rclcpp::shutdown();
  return 0;
}
