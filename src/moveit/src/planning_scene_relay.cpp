#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

class PlanningSceneRelay : public rclcpp::Node {
public:
  PlanningSceneRelay() : Node("planning_scene_relay") {
    using moveit_msgs::msg::PlanningScene;

    // MoveGroup → volatile
    rclcpp::QoS in_qos(rclcpp::KeepLast(1));
    in_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    in_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // RViz → transient_local (latched)
    rclcpp::QoS out_qos(rclcpp::KeepLast(1));
    out_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    out_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    pub_ = this->create_publisher<PlanningScene>("/monitored_planning_scene_tl", out_qos);
    sub_ = this->create_subscription<PlanningScene>(
      "/monitored_planning_scene", in_qos,
      [this](PlanningScene::ConstSharedPtr msg){ pub_->publish(*msg); });

    RCLCPP_INFO(this->get_logger(),
      "Relaying /monitored_planning_scene -> /monitored_planning_scene_tl (TRANSIENT_LOCAL)");
  }
private:
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr pub_;
  rclcpp::Subscription<moveit_msgs::msg::PlanningScene>::SharedPtr sub_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningSceneRelay>());
  rclcpp::shutdown();
  return 0;
}