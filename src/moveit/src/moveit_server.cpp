#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <std_srvs/srv/trigger.hpp>
#include "moveit/srv/vector_distance.hpp"

using namespace std::chrono_literals;
using VectorDistance = moveit::srv::VectorDistance;

class CustomServer : public rclcpp::Node
{
public:
  CustomServer()
  : Node("custom_server")
  {
    world_frame_   = declare_param<std::string>("world_frame", "panda_link0");

    table_sx_      = declare_param<double>("table_size_x", 0.60);
    table_sy_      = declare_param<double>("table_size_y", 0.50);
    table_sz_      = declare_param<double>("table_size_z", 0.40);
    table_x_       = declare_param<double>("table_x", 0.50);
    table_y_       = declare_param<double>("table_y", 0.00);
    table_z_       = declare_param<double>("table_z", table_sz_/2.0);

    box_sx_        = declare_param<double>("box_size_x", 0.06);
    box_sy_        = declare_param<double>("box_size_y", 0.06);
    box_sz_        = declare_param<double>("box_size_z", 0.05);
    box_x_         = declare_param<double>("box_x", 0.50);
    box_y_         = declare_param<double>("box_y", 0.10);
    box_z_         = declare_param<double>("box_z", 0.445);

    spawn_table_   = declare_param<bool>("spawn_table", true);
    spawn_box_     = declare_param<bool>("spawn_box",   true);
    acm_allow_all_ = declare_param<bool>("acm_allow_box_and_table", false);

    reset_world_impl();

    reset_world_srv_ = create_service<std_srvs::srv::Trigger>(
      "reset_world",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        reset_world_impl();
        res->success = true;
        res->message = "world resetlendi (table/box silindi ve parametrelere gore tekrar eklendi)";
      });

    remove_box_srv_ = create_service<std_srvs::srv::Trigger>(
      "remove_box",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        remove_if_exists("box");
        res->success = true;
        res->message = "box silindi";
      });

    spawn_box_srv_ = create_service<std_srvs::srv::Trigger>(
      "spawn_box",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        spawn_box();
        res->success = true;
        res->message = "box eklendi";
      });

    // VectorDistance: MoveGroup'e ihtiyaç var → callback içinde lazy init
    vector_distance_srv_ = create_service<VectorDistance>(
      "VectorDistance",
      [this](const std::shared_ptr<VectorDistance::Request> req,
             std::shared_ptr<VectorDistance::Response> resp)
      {
        ensure_move_group_ready(); // <— kritik: burada kuruyoruz

        RCLCPP_INFO(get_logger(), "Request: x=%.3f y=%.3f z=%.3f", req->x, req->y, req->z);

        geometry_msgs::msg::PoseStamped target;
        target.header.frame_id = world_frame_;
        target.pose.position.x = req->x;
        target.pose.position.y = req->y;
        target.pose.position.z = req->z;
        target.pose.orientation.w = 1.0;

        move_group_->clearPoseTargets();
        move_group_->setStartStateToCurrentState();
        move_group_->setPoseTarget(target);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto ok_plan = static_cast<bool>(move_group_->plan(plan));
        if (!ok_plan) {
          resp->distance = "planlama başarısız";
          RCLCPP_WARN(get_logger(), "Planlama başarısız.");
          return;
        }
        auto ok_exec = static_cast<bool>(move_group_->execute(plan));
        if (!ok_exec) {
          resp->distance = "yürütme başarısız";
          RCLCPP_WARN(get_logger(), "Yürütme başarısız.");
          return;
        }
        resp->distance = "hedefe ulaşıldı";
        RCLCPP_INFO(get_logger(), "Hedefe ulaşıldı.");
      });

    RCLCPP_INFO(get_logger(), "custom_server hazir. Servisler: /reset_world, /remove_box, /spawn_box, /VectorDistance");
  }

private:
  template<typename T>
  T declare_param(const std::string& name, const T& def)
  {
    if (!has_parameter(name)) declare_parameter<T>(name, def);
    return get_parameter(name).get_value<T>();
  }

  void ensure_move_group_ready()
  {
    if (move_group_) return;
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                    this->shared_from_this(), "panda_arm");
    RCLCPP_INFO(get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "EE link       : %s", move_group_->getEndEffectorLink().c_str());

    move_group_->setPlanningTime(3.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setMaxVelocityScalingFactor(0.4);
    move_group_->setMaxAccelerationScalingFactor(0.4);
  }

  void reset_world_impl()
  {
    remove_if_exists("box");
    remove_if_exists("table");

    if (spawn_table_) spawn_table();
    if (spawn_box_)   spawn_box();

    if (acm_allow_all_) allow_box_and_table_collisions();
  }

  void remove_if_exists(const std::string& id)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    auto objs = psi.getObjects({id});
    if (!objs.empty()) {
      psi.removeCollisionObjects({id});
      rclcpp::sleep_for(100ms);
      RCLCPP_INFO(get_logger(), "'%s' sahneden silindi.", id.c_str());
    }
  }

  void spawn_table()
  {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {table_sx_, table_sy_, table_sz_};

    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x = table_x_;
    p.position.y = table_y_;
    p.position.z = table_z_;

    moveit_msgs::msg::CollisionObject co;
    co.id = "table";
    co.header.frame_id = world_frame_;
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(co);
    rclcpp::sleep_for(100ms);
    RCLCPP_INFO(get_logger(), "TABLE eklendi (frame=%s).", world_frame_.c_str());
  }

  void spawn_box()
  {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {box_sx_, box_sy_, box_sz_};

    geometry_msgs::msg::Pose p;
    p.orientation.w = 1.0;
    p.position.x = box_x_;
    p.position.y = box_y_;
    p.position.z = box_z_;

    moveit_msgs::msg::CollisionObject co;
    co.id = "box";
    co.header.frame_id = world_frame_;
    co.primitives.push_back(prim);
    co.primitive_poses.push_back(p);
    co.operation = co.ADD;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(co);
    rclcpp::sleep_for(100ms);
    RCLCPP_INFO(get_logger(), "BOX eklendi (frame=%s, %.3f %.3f %.3f).",
                world_frame_.c_str(), box_x_, box_y_, box_z_);
  }

  void allow_box_and_table_collisions()
  {
    auto cli = create_client<moveit_msgs::srv::ApplyPlanningScene>("/apply_planning_scene");
    if (!cli->wait_for_service(3s)) {
      RCLCPP_WARN(get_logger(), "apply_planning_scene servisi yok; ACM atlandi.");
      return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    req->scene.is_diff = true;
    req->scene.allowed_collision_matrix.default_entry_names  = {"box", "table"};
    req->scene.allowed_collision_matrix.default_entry_values = {true,  true};

    auto fut = cli->async_send_request(req);
    if (fut.wait_for(2s) == std::future_status::ready && fut.get()->success)
      RCLCPP_INFO(get_logger(), "ACM: box/table collision allow ACILDI.");
    else
      RCLCPP_WARN(get_logger(), "ACM gonderimi basarisiz.");
  }

private:
  std::string world_frame_;

  double table_sx_, table_sy_, table_sz_;
  double table_x_, table_y_, table_z_;
  
  double box_sx_, box_sy_, box_sz_;
  double box_x_, box_y_, box_z_;
  
  bool spawn_table_{true}, spawn_box_{true}, acm_allow_all_{false};


  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;


  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_world_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_box_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr spawn_box_srv_;
  rclcpp::Service<VectorDistance>::SharedPtr          vector_distance_srv_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CustomServer>());
  rclcpp::shutdown();
  return 0;
}
