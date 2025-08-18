#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

#include "moveit/srv/vector_distance.hpp"

using namespace std::chrono_literals;
using VectorDistance = moveit::srv::VectorDistance;

// Dünya objelerini oluştur (masa + kutu)
static std::vector<moveit_msgs::msg::CollisionObject> makeWorldObjects()
{
  std::vector<moveit_msgs::msg::CollisionObject> objs;

  // ---- TABLE ----
  {
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {0.6, 0.5, 0.4}; // x y z (m)

    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x = 0.50;
    p.position.y = 0.00;
    p.position.z = 0.20; // yükseklik/2

    table.primitives.push_back(prim);
    table.primitive_poses.push_back(p);
    table.operation = table.ADD;
    objs.push_back(table);
  }

  // ---- BOX ----
  {
    moveit_msgs::msg::CollisionObject box;
    box.id = "box";
    box.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {0.04, 0.04, 0.12};

    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x = 0.50;
    p.position.y = 0.10;
    p.position.z = 0.46;

    box.primitives.push_back(prim);
    box.primitive_poses.push_back(p);
    box.operation = box.ADD;
    objs.push_back(box);
  }

  return objs;
}

// ACM'de 'box' ve 'table' için çarpışmayı komple aç
static void allowBoxAndTableCollisions(const rclcpp::Node::SharedPtr& node)
{
  auto client = node->create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
  if (!client->wait_for_service(3s))
  {
    RCLCPP_WARN(node->get_logger(), "apply_planning_scene servisi 3sn içinde gelmedi, ACM ayari atlanacak.");
    return;
  }

  auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
  req->scene.is_diff = true;
  req->scene.allowed_collision_matrix.default_entry_names = {"box", "table"};
  req->scene.allowed_collision_matrix.default_entry_values = {true, true};

  auto fut = client->async_send_request(req);
  auto ret = fut.wait_for(2s);
  if (ret == std::future_status::ready && fut.get()->success)
    RCLCPP_INFO(node->get_logger(), "ACM: 'box' ve 'table' icin collision ALLOW ayarlandi.");
  else
    RCLCPP_WARN(node->get_logger(), "ACM gonderimi basarisiz ya da yanit gelmedi.");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("custom_server");

  // MoveGroup & PSI
  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "panda_arm");
  moveit::planning_interface::PlanningSceneInterface psi;

  RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str());
  RCLCPP_INFO(node->get_logger(), "EE link: %s", move_group->getEndEffectorLink().c_str());

  // Dünya objelerini uygula
  {
    auto objs = makeWorldObjects();
    psi.applyCollisionObjects(objs);
    RCLCPP_INFO(node->get_logger(), "Masa ve kutu sahneye eklendi.");
  }

  // ACM: box & table ile tum collision'lari ALLOW
  allowBoxAndTableCollisions(node);

  // Planlama ayarlari
  move_group->setPlanningTime(3.0);
  move_group->setNumPlanningAttempts(5);
  move_group->setMaxVelocityScalingFactor(0.4);
  move_group->setMaxAccelerationScalingFactor(0.4);

  // Servis: /VectorDistance
  auto srv = node->create_service<VectorDistance>(
      "VectorDistance",
      [node, move_group](const std::shared_ptr<VectorDistance::Request> req,
                         std::shared_ptr<VectorDistance::Response> resp)
      {
        RCLCPP_INFO(node->get_logger(), "Request: x=%.3f, y=%.3f, z=%.3f", req->x, req->y, req->z);

        // Hedef pozu (dunya frame'i)
        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = "world";
        target.pose.position.x = req->x;
        target.pose.position.y = req->y;
        target.pose.position.z = req->z;
        target.pose.orientation.w = 1.0; // düz bakış

        move_group->clearPoseTargets();
        move_group->setStartStateToCurrentState();
        move_group->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto rc_plan = move_group->plan(plan);

        // MoveIt 2'de plan() sonucu MoveItErrorCode ise bool'a çevrilebilir
        bool planned = static_cast<bool>(rc_plan);
        if (!planned)
        {
          RCLCPP_WARN(node->get_logger(), "Planlama basarisiz.");
          resp->distance = "planlama başarısız";
          return;
        }

        RCLCPP_INFO(node->get_logger(), "Plan bulundu, yürütülüyor...");
        auto rc_exec = move_group->execute(plan);
        bool exec_ok = static_cast<bool>(rc_exec);

        if (!exec_ok)
        {
          RCLCPP_WARN(node->get_logger(), "Yurutme basarisiz.");
          resp->distance = "yürütme başarısız";
          return;
        }

        RCLCPP_INFO(node->get_logger(), "Hedefe ulasildi.");
        resp->distance = "hedefe ulaşıldı";
      });

  RCLCPP_INFO(node->get_logger(), "Service hazir: VectorDistance");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
