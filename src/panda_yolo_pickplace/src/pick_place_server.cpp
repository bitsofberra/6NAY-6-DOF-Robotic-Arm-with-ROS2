#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <panda_yolo_pickplace/srv/trigger_pick_place.hpp>

// MoveIt headers (new .hpp or old .h)
#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
  #include <moveit/move_group_interface/move_group_interface.hpp>
#else
  #include <moveit/move_group_interface/move_group_interface.h>
#endif

#if __has_include(<moveit/planning_scene_interface/planning_scene_interface.hpp>)
  #include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#else
  #include <moveit/planning_scene_interface/planning_scene_interface.h>
#endif

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>


using TriggerPickPlace = panda_yolo_pickplace::srv::TriggerPickPlace;

class PickPlaceServer : public rclcpp::Node {
public:
  PickPlaceServer(): Node("pick_place_server"), arm_(shared_from_this(), "panda_arm"), hand_(shared_from_this(), "hand"){
    sub_red_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/red_block/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){ last_red_ = *m; have_red_ = true; });
    sub_green_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/green_block/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr m){ last_green_ = *m; have_green_ = true; });
    srv_ = this->create_service<TriggerPickPlace>("/pick_place/trigger",
      std::bind(&PickPlaceServer::onCall, this, std::placeholders::_1, std::placeholders::_2));

    arm_.setPlanningTime(5.0); arm_.setMaxVelocityScalingFactor(0.5); arm_.setMaxAccelerationScalingFactor(0.5);
  }

private:
  void onCall(const std::shared_ptr<TriggerPickPlace::Request> req,
              std::shared_ptr<TriggerPickPlace::Response> res){
    geometry_msgs::msg::PoseStamped obj;
    std::string color = req->color;
    if(color=="red" && have_red_) obj = last_red_;
    else if(color=="green" && have_green_) obj = last_green_;
    else { res->success=false; res->message="No detection for "+color; return; }

    // pre-grasp above object
    auto pre = obj; pre.pose.position.z += 0.12;
    // grasp pose
    auto grasp = obj; grasp.pose.position.z += 0.01; // near top surface

    // Open gripper
    controlHand(0.04);

    if(!planExec(pre)) { res->success=false; res->message="Pre-grasp plan failed"; return; }
    if(!planExec(grasp)) { res->success=false; res->message="Grasp plan failed"; return; }

    // Close gripper
    controlHand(0.002);

    // Attach virtual collision object to the gripper to avoid self-collision in place motion
    attachBox("carried_block");

    // retreat
    auto retreat = grasp; retreat.pose.position.z += 0.15; if(!planExec(retreat)){ res->success=false; res->message="Retreat failed"; return; }

    // place pose (choose by color)
    geometry_msgs::msg::PoseStamped place; place.header.frame_id = "world"; place.pose.orientation.w=1.0;
    if(color=="red"){ place.pose.position.x = 0.90; place.pose.position.y= 0.15; place.pose.position.z=0.80; }
    else { place.pose.position.x = 0.90; place.pose.position.y=-0.15; place.pose.position.z=0.80; }

    if(!planExec(place)){ res->success=false; res->message="Place plan failed"; return; }

    // open and detach
    controlHand(0.04); detachBox("carried_block");

    // home
    arm_.setNamedTarget("ready"); arm_.move();

    res->success=true; res->message="Done";
  }

  bool planExec(const geometry_msgs::msg::PoseStamped &target){ arm_.setPoseTarget(target); moveit::planning_interface::MoveGroupInterface::Plan p; bool ok = (arm_.plan(p) == moveit::core::MoveItErrorCode::SUCCESS); if(ok) arm_.execute(p); return ok; }

  void controlHand(double width){
    // For Franka hand: single joint control by MoveGroup "hand" via joint value target
    const std::vector<std::string> js = hand_.getJointNames();
    std::map<std::string, double> goals;
    for(const auto &j: js) goals[j] = width; // panda_finger_joint1 mirrored internally
    hand_.setJointValueTarget(goals); hand_.move();
  }

  void attachBox(const std::string &id){
    moveit_msgs::msg::CollisionObject obj; obj.header.frame_id = "panda_hand"; obj.id = id;
    shape_msgs::msg::SolidPrimitive box; box.type=box.BOX; box.dimensions={0.04,0.04,0.04};
    geometry_msgs::msg::Pose p; p.orientation.w=1.0; obj.primitives.push_back(box); obj.primitive_poses.push_back(p); obj.operation = obj.ADD;
    psi_.applyCollisionObject(obj);
    arm_.attachObject(id, "panda_hand");
  }
  void detachBox(const std::string &id){ arm_.detachObject(id); psi_.removeCollisionObjects({id}); }

  rclcpp::Service<TriggerPickPlace>::SharedPtr srv_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_red_, sub_green_;
  geometry_msgs::msg::PoseStamped last_red_, last_green_; bool have_red_=false, have_green_=false;
  moveit::planning_interface::MoveGroupInterface arm_, hand_;
  moveit::planning_interface::PlanningSceneInterface psi_;
};

int main(int argc, char** argv){ rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<PickPlaceServer>()); rclcpp::shutdown(); return 0; }