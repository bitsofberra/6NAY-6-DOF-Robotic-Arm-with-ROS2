#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("demo_pick_place");

  // Not: Jazzy’de genelde /clock var; sim zamanı kullan
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));

  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/panda_arm_controller/joint_trajectory", 10);

  // Ortak eklem isimleri
  std::vector<std::string> joints = {
    "panda_joint1","panda_joint2","panda_joint3","panda_joint4",
    "panda_joint5","panda_joint6","panda_joint7"
  };

  // Mesaj
  trajectory_msgs::msg::JointTrajectory traj;
  traj.joint_names = joints;

  auto mk = [](std::initializer_list<double> v, double t)->trajectory_msgs::msg::JointTrajectoryPoint{
    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = v;
    p.time_from_start = rclcpp::Duration::from_seconds(t);
    return p;
  };

  // Basit bir senaryo: home -> pregrasp -> grasp (aynı) -> place -> home
  traj.points.push_back(mk({0, -0.4, 0, -2.0, 0, 1.6, 0.8}, 2.0));  // home benzeri
  traj.points.push_back(mk({0.2, -0.6, 0.2, -2.2, 0.1, 1.8, 0.6}, 4.0)); // pregrasp
  traj.points.push_back(mk({0.25, -0.7, 0.25, -2.3, 0.15, 1.9, 0.5}, 6.0)); // "grasp"
  traj.points.push_back(mk({-0.3, -0.5, 0.1, -1.8, -0.2, 1.4, 0.9}, 8.5)); // place
  traj.points.push_back(mk({0, -0.4, 0, -2.0, 0, 1.6, 0.8}, 11.0)); // home

  // Biraz bekle ki subscriber hazır olsun
  rclcpp::sleep_for(500ms);

  RCLCPP_INFO(node->get_logger(), "Gönderiliyor: %zu nokta", traj.points.size());
  pub->publish(traj);

  // Yörünge bitene kadar node’u tut
  rclcpp::sleep_for(12s);
  rclcpp::shutdown();
  return 0;
}
