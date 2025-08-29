// pubsub_srvcli/src/server_moveit_panda.cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

#include <nlohmann/json.hpp>
#include "pubsub_srvcli/srv/vector_distance.hpp"
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
using VectorDistance = pubsub_srvcli::srv::VectorDistance;
using json = nlohmann::json;

static bool have_param(const rclcpp::Node::SharedPtr& node, const char* name) {
  rclcpp::Parameter p;
  return node->has_parameter(name) || node->get_parameter(name, p);
}
static void declare_or_set(rclcpp::Node::SharedPtr node, const rclcpp::Parameter& p) {
  if (p.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) return;
  const auto& name = p.get_name();
  const auto& val  = p.get_parameter_value();
  if (!node->has_parameter(name)) node->declare_parameter(name, val);
  else node->set_parameter(p);
}
static void copy_moveit_params_from_move_group(const rclcpp::Node::SharedPtr& node) {
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
  RCLCPP_INFO(node->get_logger(), "Waiting for /move_group parameter service...");
  if (!client->wait_for_service(20s)) {
    RCLCPP_WARN(node->get_logger(), "move_group param service yok; paramlar launch ile gelmeli.");
    return;
  }
  std::vector<std::string> names = {
    "robot_description","robot_description_semantic","robot_description_kinematics",
    "robot_description_planning","robot_description_planning_pipelines",
    "planning_pipelines","trajectory_execution","moveit_controller_manager"
  };
  std::vector<rclcpp::Parameter> got;
  try { got = client->get_parameters(names); } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "Param okunamadı: %s", e.what()); return; }
  size_t n = 0; for (const auto& p : got) { if (p.get_type()==rclcpp::ParameterType::PARAMETER_NOT_SET) continue;
    declare_or_set(node, p); ++n; }
  RCLCPP_INFO(node->get_logger(), "move_group'tan %zu MoveIt paramı kopyalandı.", n);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts; opts.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("panda_move_server", opts);

  std::string json_path;
  if (!node->get_parameter("json_path", json_path)) {
    json_path = "/home/revengeofthesob/ros3_ws/src/pubsub_srvcli/src/veri.json";
    node->declare_parameter<std::string>("json_path", json_path);
  }

  if (!have_param(node, "robot_description") || !have_param(node, "robot_description_semantic"))
    copy_moveit_params_from_move_group(node);

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while ((!have_param(node, "robot_description") || !have_param(node, "robot_description_semantic")) &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::sleep_for(200ms);
  }

  std::string urdf, srdf;
  if (have_param(node, "robot_description")) urdf = node->get_parameter("robot_description").as_string();
  if (have_param(node, "robot_description_semantic")) srdf = node->get_parameter("robot_description_semantic").as_string();
  if (urdf.empty() || srdf.empty()) {
    RCLCPP_FATAL(node->get_logger(), "MoveIt paramları eksik. urdf_len=%zu srdf_len=%zu", urdf.size(), srdf.size());
    rclcpp::shutdown(); return 1;
  }

  using MGI = moveit::planning_interface::MoveGroupInterface;
  MGI::Options opt("panda_arm", "robot_description");
  auto move_group = std::make_shared<MGI>(node, opt);
  move_group->setEndEffectorLink("panda_link8");
  move_group->setPlanningTime(5.0);
  move_group->setMaxVelocityScalingFactor(0.30);
  move_group->setMaxAccelerationScalingFactor(0.30);

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "EEF link: %s", move_group->getEndEffectorLink().c_str());

  // ---- SERVİSLER (private adlar: ~/...) ----
  node->create_service<VectorDistance>(
    "~/calculate_distance",
    [node, move_group](const std::shared_ptr<VectorDistance::Request> req,
                       std::shared_ptr<VectorDistance::Response> res)
    {
      geometry_msgs::msg::Pose target; target.orientation.w = 1.0;
      target.position.x = req->x; target.position.y = req->y; target.position.z = req->z;
      move_group->clearPoseTargets(); move_group->setStartStateToCurrentState(); move_group->setPoseTarget(target);
      RCLCPP_INFO(node->get_logger(), "Planning to x=%.3f y=%.3f z=%.3f in frame '%s'",
                  req->x, req->y, req->z, move_group->getPlanningFrame().c_str());
      auto code = move_group->move();
      if (code == moveit::core::MoveItErrorCode::SUCCESS) RCLCPP_INFO(node->get_logger(), "Motion success.");
      else RCLCPP_ERROR(node->get_logger(), "Motion failed (code=%d).", code.val);
      res->distance = std::sqrt(req->x*req->x + req->y*req->y + req->z*req->z);
    }
  );

  node->create_service<std_srvs::srv::Trigger>(
    "~/go_home",
    [node, move_group](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      bool ok=false; try { ok = move_group->setNamedTarget("ready"); } catch (...) { ok=false; }
      if (!ok) { try { ok = move_group->setNamedTarget("home"); } catch (...) { ok=false; } }
      if (!ok) { std::vector<double> joints = {0.0,-0.785398,0.0,-2.35619,0.0,1.5708,0.785398}; move_group->setJointValueTarget(joints); }
      move_group->setStartStateToCurrentState();
      auto code = move_group->move();
      res->success = (code == moveit::core::MoveItErrorCode::SUCCESS);
      res->message = res->success ? "Returned to home/ready." : "Failed to go home.";
      if (!res->success) RCLCPP_ERROR(node->get_logger(), "go_home failed (code=%d).", code.val);
    }
  );

  node->create_service<std_srvs::srv::Trigger>(
    "~/process_last_target_from_json",
    [node, move_group, &json_path](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      node->get_parameter("json_path", json_path);
      std::ifstream f(json_path);
      if (!f.is_open()) { res->success=false; res->message="JSON not found"; return; }
      json j; try { f >> j; } catch (...) { res->success=false; res->message="JSON parse error"; return; }
      if (!j.is_array() || j.empty()) { res->success=false; res->message="JSON empty"; return; }
      const auto &it = j.back();
      geometry_msgs::msg::Pose target; target.orientation.w = 1.0;
      try {
        target.position.x = it.at("x").get<double>();
        target.position.y = it.at("y").get<double>();
        target.position.z = it.at("z").get<double>();
      } catch (...) { res->success=false; res->message="JSON missing x/y/z"; return; }
      move_group->clearPoseTargets(); move_group->setStartStateToCurrentState(); move_group->setPoseTarget(target);
      auto code = move_group->move();
      res->success = (code == moveit::core::MoveItErrorCode::SUCCESS);
      res->message = res->success ? "Moved to last JSON target" : "Move failed";
    }
  );

  const std::string ns=node->get_namespace(), nm=node->get_name();
  const std::string base = (ns.empty()||ns=="/") ? ("/"+nm) : (ns+"/"+nm);
  RCLCPP_INFO(node->get_logger(),
    "Services:\n  %s/calculate_distance\n  %s/go_home\n  %s/process_last_target_from_json",
    base.c_str(), base.c_str(), base.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
