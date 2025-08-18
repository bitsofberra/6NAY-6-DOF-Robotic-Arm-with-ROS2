#include <memory>
#include <vector>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using Gripper = control_msgs::action::GripperCommand;

static const char* ARM_GROUP = "panda_arm";
static const char* EE_LINK   = "panda_link8";
static const char* BOX_ID    = "box";

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("pick_place_node")
  {
    arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), ARM_GROUP);
    arm_->setEndEffectorLink(EE_LINK);
    arm_->setPoseReferenceFrame("world");
    arm_->setMaxVelocityScalingFactor(0.3);
    arm_->setMaxAccelerationScalingFactor(0.3);
    arm_->setPlanningTime(5.0);
    arm_->setNumPlanningAttempts(3);

    gripper_client_ = rclcpp_action::create_client<Gripper>(this, "/panda_hand_controller/gripper_cmd");

    sub_ = create_subscription<std_msgs::msg::Empty>(
      "/pick_place/start", 10, std::bind(&PickPlaceNode::onStart, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Pick&Place hazir (topic: /pick_place/start).");
  }

private:
  bool waitForBox(double timeout_s = 5.0)
  {
    auto start = now();
    while ((now() - start).seconds() < timeout_s) {
      auto m = psi_.getObjects({BOX_ID});
      if (!m.empty()) return true;
      rclcpp::sleep_for(100ms);
    }
    RCLCPP_ERROR(get_logger(), "Planning scene'de '%s' bulunamadi (%.1fs).", BOX_ID, timeout_s);
    return false;
  }

  bool planAndExecToPose(const geometry_msgs::msg::Pose& p)
  {
    arm_->setPoseTarget(p);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (arm_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      return false;
    }
    return (arm_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
  }

  bool gripperCommand(double position, double effort = 40.0)
  {
    if (!gripper_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(get_logger(), "Gripper action server yok.");
      return false;
    }
    Gripper::Goal goal;
    goal.command.position = position;  // 0.0 kapalı, ~0.04 açık (fake için tipik)
    goal.command.max_effort = effort;

    auto future_handle = gripper_client_->async_send_goal(goal);
    auto gh_status = rclcpp::spin_until_future_complete(get_node_base_interface(), future_handle, 5s);
    if (gh_status != rclcpp::FutureReturnCode::SUCCESS) return false;

    auto handle = future_handle.get();
    if (!handle) return false;

    auto result_future = gripper_client_->async_get_result(handle);
    auto res_status = rclcpp::spin_until_future_complete(get_node_base_interface(), result_future, 10s);
    return (res_status == rclcpp::FutureReturnCode::SUCCESS);
  }

  void onStart(const std_msgs::msg::Empty &)
  {
    RCLCPP_INFO(get_logger(), "Pick&Place tetiklendi");

    if (!waitForBox(5.0)) return;

    // Kutu merkez pozunu al
    auto objs = psi_.getObjects({BOX_ID});
    const auto& co = objs.begin()->second;
    if (co.primitive_poses.empty()) {
      RCLCPP_ERROR(get_logger(), "Box primitive_poses bos!");
      return;
    }
    geometry_msgs::msg::Pose box_pose = co.primitive_poses.front();

    // Yaklaşma / tutma / taşıma / bırakma pozları
    geometry_msgs::msg::Pose approach = box_pose;
    approach.position.z += 0.10;           // 10cm üstünden yaklaş

    geometry_msgs::msg::Pose grasp = box_pose;
    grasp.position.z += 0.02;              // hafifçe üstten

    geometry_msgs::msg::Pose lift = grasp;
    lift.position.z += 0.15;               // kaldır

    // basit bırakma noktası (kutunun x+0.2 yanına bırak)
    geometry_msgs::msg::Pose place_approach = approach;
    place_approach.position.x += 0.20;

    geometry_msgs::msg::Pose place = grasp;
    place.position.x += 0.20;

    // 1) Aç
    if (!gripperCommand(0.04)) {
      RCLCPP_WARN(get_logger(), "Gripper open basarisiz (devam).");
    }

    // 2) Yaklaş
    if (!planAndExecToPose(approach)) {
      RCLCPP_ERROR(get_logger(), "Approach plan basarisiz");
      return;
    }

    // 3) İn
    if (!planAndExecToPose(grasp)) {
      RCLCPP_ERROR(get_logger(), "Grasp plan basarisiz");
      return;
    }

    // 4) Kapat
    if (!gripperCommand(0.0)) {
      RCLCPP_WARN(get_logger(), "Gripper close basarisiz (devam).");
    }

    // 5) Objeyi ataçla (collision açısından)
    std::vector<std::string> touch = {"panda_hand","panda_finger_link1","panda_finger_link2"};
    arm_->attachObject(BOX_ID, EE_LINK, touch);

    // 6) Kaldır
    if (!planAndExecToPose(lift)) {
      RCLCPP_ERROR(get_logger(), "Lift plan basarisiz");
      return;
    }

    // 7) Bırakma yaklaşma
    if (!planAndExecToPose(place_approach)) {
      RCLCPP_ERROR(get_logger(), "Place approach basarisiz");
      return;
    }

    // 8) İn
    if (!planAndExecToPose(place)) {
      RCLCPP_ERROR(get_logger(), "Place indiş basarisiz");
      return;
    }

    // 9) Aç
    if (!gripperCommand(0.04)) {
      RCLCPP_WARN(get_logger(), "Gripper open (place) basarisiz (devam).");
    }

    // 10) Detach
    arm_->detachObject(BOX_ID);

    // 11) Yukarı kalk
    if (!planAndExecToPose(place_approach)) {
      RCLCPP_WARN(get_logger(), "Place yukari donus plan basarisiz");
    }

    RCLCPP_INFO(get_logger(), "Pick&Place tamam.");
  }

  moveit::planning_interface::PlanningSceneInterface psi_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  rclcpp_action::Client<Gripper>::SharedPtr gripper_client_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PickPlaceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
