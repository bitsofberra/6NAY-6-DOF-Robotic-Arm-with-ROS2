#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include "moveit/srv/vector_distance.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("server_custom_msg");

  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

  moveit::planning_interface::PlanningSceneInterface psi;

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "EE link: %s", move_group.getEndEffectorLink().c_str());

  std::vector<moveit_msgs::msg::CollisionObject> objs;

  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "table";
  table.pose.orientation.w = 1.0;
  table.pose.position.x = 0.5;
  table.pose.position.y = 0.0;
  table.pose.position.z = 0.2;
  table.primitives.resize(1);
  table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  table.primitives[0].dimensions = {0.6, 0.5, 0.4};
  table.primitive_poses.resize(1);
  table.primitive_poses[0].orientation.w = 1.0;
  objs.push_back(table);

  moveit_msgs::msg::CollisionObject box;
  box.header.frame_id = "world";
  box.id = "box";
  box.pose.orientation.w = 1.0;
  box.pose.position.x = 0.5;
  box.pose.position.y = 0.1;
  box.pose.position.z = 0.46;
  box.primitives.resize(1);
  box.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  box.primitives[0].dimensions = {0.04, 0.04, 0.12};
  box.primitive_poses.resize(1);
  box.primitive_poses[0].orientation.w = 1.0;
  objs.push_back(box);

  psi.applyCollisionObjects(objs);
  RCLCPP_INFO(node->get_logger(), "Masa ve kutu sahneye uygulandi.");

  auto srv = node->create_service<moveit::srv::VectorDistance>(
    "/VectorDistance",
    [&node, &move_group](const std::shared_ptr<moveit::srv::VectorDistance::Request> req,
                         std::shared_ptr<moveit::srv::VectorDistance::Response> res)
    {
      auto cs = move_group.getCurrentState(2.0);
      if (!cs) {
        RCLCPP_WARN(node->get_logger(), "Current robot state ALINAMADI; start state current kabul ediliyor.");
      }
      move_group.setStartStateToCurrentState();

      geometry_msgs::msg::Pose target;
      target.position.x = req->x;
      target.position.y = req->y;
      target.position.z = req->z;
      target.orientation.w = 1.0;

      move_group.setPoseTarget(target);
      move_group.setPlanningTime(5.0);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (!ok) {
        res->distance = "planlama başarısız";
        RCLCPP_WARN(node->get_logger(), "Planlama başarısız (x=%.2f y=%.2f z=%.2f)", req->x, req->y, req->z);
        move_group.clearPoseTargets();
        return;
      }

      auto ex = move_group.execute(plan);
      if (ex != moveit::core::MoveItErrorCode::SUCCESS) {
        res->distance = "yürütme başarısız";
        RCLCPP_WARN(node->get_logger(), "Yürütme başarısız");
      } else {
        res->distance = "hedefe ulaşıldı";
      }
      move_group.clearPoseTargets();
    });

  RCLCPP_INFO(node->get_logger(), "Service hazir: /VectorDistance");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
