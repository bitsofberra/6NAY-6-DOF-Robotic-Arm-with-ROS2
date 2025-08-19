#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

class PickPlaceNode : public rclcpp::Node
{
public:
  PickPlaceNode()
  : Node("pick_place_node",
         rclcpp::NodeOptions()
           .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
  {
    planning_group_   = this->declare_parameter<std::string>("planning_group", "panda_arm");
    eef_link_         = this->declare_parameter<std::string>("eef_link", "panda_link8");
    world_frame_      = this->declare_parameter<std::string>("world_frame", "world");

    approach_distance_ = this->declare_parameter<double>("approach_distance", 0.18);
    retreat_distance_  = this->declare_parameter<double>("retreat_distance", 0.20);
    place_dx_          = this->declare_parameter<double>("place_dx", -0.20);
    place_dy_          = this->declare_parameter<double>("place_dy", -0.20);

    RCLCPP_INFO(get_logger(), "PickPlaceNode hazir. group=%s eef=%s frame=%s",
                planning_group_.c_str(), eef_link_.c_str(), world_frame_.c_str());
  }

  int run()
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    auto self = this->shared_from_this();
    exec.add_node(self);

    if (!mirrorMoveItParams(exec)) {
      RCLCPP_FATAL(get_logger(), "FATAL in run(): could not mirror MoveIt params");
      return 2;
    }
    RCLCPP_INFO(get_logger(), "URDF/SRDF (+kinematics/planning) paramlari mirror edildi.");

    moveit::planning_interface::MoveGroupInterface mgi(self, planning_group_);
    mgi.setPoseReferenceFrame(world_frame_);
    mgi.setEndEffectorLink(eef_link_);
    mgi.setPlanningTime(15.0);
    mgi.setNumPlanningAttempts(32);
    mgi.setMaxVelocityScalingFactor(0.4);
    mgi.setMaxAccelerationScalingFactor(0.4);
    mgi.setGoalPositionTolerance(0.01);    // 1 cm
    mgi.setGoalOrientationTolerance(0.35); // ~20°
    mgi.setGoalJointTolerance(0.01);
    mgi.setPlannerId("RRTConnectkConfigDefault");
    mgi.setWorkspace(-1.0, -1.0, 0.0, 1.5, 1.5, 1.5);

    RCLCPP_INFO(get_logger(), "Group=%s | EEF=%s | Frame=%s",
                planning_group_.c_str(), eef_link_.c_str(), world_frame_.c_str());

    trySetNamed(mgi, "ready");

    // Objeyi bul (scene -> psi -> default)
    ObjInfo box = queryObjectRobust("box", exec);
    if (!box.valid) {
      RCLCPP_FATAL(get_logger(), "Box bilgisi bulunamadi (sahne/psi/default).");
      return 3;
    }
    RCLCPP_INFO(get_logger(), "[src=%s] Obj '%s' -> pick=(%.3f,%.3f,%.3f)",
                box.source.c_str(), box.id.c_str(),
                box.center.position.x, box.center.position.y, box.center.position.z);

    // Boy ve Z seviyeleri
    const double half_h = (box.size_z > 1e-6 ? box.size_z*0.5 : 0.06);  // bilinmiyorsa 12cm varsay
    const double table_top_z = 0.40;
    const double box_top_z   = box.center.position.z + half_h;

    // Hedef noktalar
    const double place_cx = box.center.position.x + place_dx_;
    const double place_cy = box.center.position.y + place_dy_;

    // Aşağı bakan sabit quaternion (180° X etrafında): w=0, x=1, y=0, z=0
    auto q_down = [](){
      geometry_msgs::msg::Quaternion q; q.w=0.0; q.x=1.0; q.y=0.0; q.z=0.0; return q;
    }();

    auto pose_at = [&](double x,double y,double z){
      geometry_msgs::msg::Pose p; p.position.x=x; p.position.y=y; p.position.z=z; p.orientation=q_down; return p;
    };

    // pre-pick (yüksek), pick (tepe + güvenli pay)
    const geometry_msgs::msg::Pose pre_pick  = pose_at(box.center.position.x, box.center.position.y,
                                                       box_top_z + approach_distance_);
    const geometry_msgs::msg::Pose pick      = pose_at(box.center.position.x, box.center.position.y,
                                                       box_top_z + 0.05);  // **tepe + 5cm**

    // pre-place (yüksek), place (tepe + güvenli pay)
    const geometry_msgs::msg::Pose pre_place = pose_at(place_cx, place_cy,
                                                       table_top_z + half_h + approach_distance_);
    const geometry_msgs::msg::Pose place     = pose_at(place_cx, place_cy,
                                                       table_top_z + half_h + 0.05);

    // 1) pre-pick: tam poz hedefi
    if (!planAndExecutePose(mgi, pre_pick))   return fail("Plan bulunamadi (pre-pick).");

    // 2) pick: **Cartesian düz inis** (fallback: normal plan)
    if (!descendCartesian(mgi, pre_pick, pick))
      if (!planAndExecutePose(mgi, pick))     return fail("Plan bulunamadi (pick).");

    // 3) pre-place: normal plan
    if (!planAndExecutePose(mgi, pre_place))  return fail("Plan bulunamadi (pre-place).");

    // 4) place: **Cartesian düz inis** (fallback: normal plan)
    if (!descendCartesian(mgi, pre_place, place))
      if (!planAndExecutePose(mgi, place))    return fail("Plan bulunamadi (place).");

    RCLCPP_INFO(get_logger(), "Pick & Place BASARILI.");
    return 0;
  }

private:
  struct ObjInfo {
    bool valid{false};
    std::string id;
    geometry_msgs::msg::Pose center{};
    double size_x{0}, size_y{0}, size_z{0};
    std::string source; // "scene" / "psi" / "default"
  };

  // ---------- Param mirroring ----------
  bool mirrorMoveItParams(rclcpp::executors::SingleThreadedExecutor &exec)
  {
    std::string urdf, srdf, who_urdf, who_srdf;

    const std::vector<std::string> prefer_urdf = {
      "/move_group","move_group", "/robot_state_publisher","robot_state_publisher", "/rviz2","rviz2"
    };
    const std::vector<std::string> prefer_srdf = {
      "/move_group","move_group", "/rviz2","rviz2"
    };

    const auto deadline = this->now() + rclcpp::Duration(90,0);

    while (this->now() < deadline) {
      if (discoverParamString(exec, prefer_urdf, "robot_description", urdf, who_urdf)) break;
      rclcpp::sleep_for(1000ms);
    }
    if (urdf.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not get URDF from candidates (robot_description)");
      return false;
    }

    while (this->now() < deadline) {
      if (discoverParamString(exec, prefer_srdf, "robot_description_semantic", srdf, who_srdf)) break;
      rclcpp::sleep_for(1000ms);
    }
    if (srdf.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not get SRDF from candidates (robot_description_semantic)");
      return false;
    }

    this->set_parameters({
      rclcpp::Parameter("robot_description", urdf),
      rclcpp::Parameter("robot_description_semantic", srdf)
    });

    size_t kopied_kine = mirrorPrefix(exec, "/move_group", "robot_description_kinematics");
    size_t kopied_plan = mirrorPrefix(exec, "/move_group", "robot_description_planning");
    RCLCPP_INFO(get_logger(), "[/move_group] %zu param kopyalandi (prefix=robot_description_kinematics).", kopied_kine);
    RCLCPP_INFO(get_logger(), "[/move_group] %zu param kopyalandi (prefix=robot_description_planning).",   kopied_plan);

    return true;
  }

  size_t mirrorPrefix(rclcpp::executors::SingleThreadedExecutor &exec,
                      const std::string &remote_node, const std::string &prefix)
  {
    auto cli = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this(), remote_node);
    if (!cli->wait_for_service(3000ms)) return 0;

    auto fut_list = cli->list_parameters({prefix}, 100);
    if (exec.spin_until_future_complete(fut_list, 3000ms) != rclcpp::FutureReturnCode::SUCCESS) return 0;
    auto listed = fut_list.get();
    if (listed.names.empty()) return 0;

    auto fut_get = cli->get_parameters(listed.names);
    if (exec.spin_until_future_complete(fut_get, 4000ms) != rclcpp::FutureReturnCode::SUCCESS) return 0;
    auto values = fut_get.get();

    this->set_parameters(values);
    return values.size();
  }

  bool discoverParamString(rclcpp::executors::SingleThreadedExecutor &exec,
                           const std::vector<std::string>& preferred_nodes,
                           const std::string &key, std::string &out, std::string &who)
  {
    std::vector<std::string> candidates;
    for (auto &n : preferred_nodes)
      if (std::find(candidates.begin(), candidates.end(), n) == candidates.end())
        candidates.push_back(n);

    for (int pass=0; pass<5; ++pass) {
      for (const auto &full : candidates) {
        if (tryGetStringParamFromNode(exec, full, key, out)) {
          who = full;
          return true;
        }
      }
      rclcpp::sleep_for(1000ms);
    }
    return false;
  }

  bool tryGetStringParamFromNode(rclcpp::executors::SingleThreadedExecutor &exec,
                                 const std::string &remote_node,
                                 const std::string &key,
                                 std::string &out)
  {
    auto cli = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this(), remote_node);
    if (!cli->wait_for_service(3000ms)) return false;

    auto fut = cli->get_parameters({key});
    if (exec.spin_until_future_complete(fut, 3000ms) != rclcpp::FutureReturnCode::SUCCESS) return false;

    auto result = fut.get();
    if (result.empty()) return false;
    if (result[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) return false;

    out = result[0].as_string();
    return !out.empty();
  }

  // ---------- Obje sorgu (robust) ----------
  ObjInfo queryObjectRobust(const std::string &id, rclcpp::executors::SingleThreadedExecutor &exec)
  {
    for (int i=1; i<=10; ++i) {
      ObjInfo s = queryObjectFromScene(id, exec);
      if (s.valid && !isZeroPose(s.center)) { s.source="scene"; return s; }
      if (s.valid && isZeroPose(s.center))
        RCLCPP_WARN(get_logger(), "Obj '%s' sahnede ama poz (0,0,0) gorunuyor (try %d/10) — tekrar denenecek.", id.c_str(), i);
      rclcpp::sleep_for(800ms);
    }
    ObjInfo p = queryObjectFromPSI(id);
    if (p.valid && !isZeroPose(p.center)) { p.source="psi"; return p; }

    ObjInfo d = serverDefaults(id);
    if (d.valid) { d.source="default"; return d; }

    return ObjInfo{};
  }

  ObjInfo queryObjectFromScene(const std::string &id, rclcpp::executors::SingleThreadedExecutor &exec)
  {
    auto cli = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!cli->wait_for_service(3s)) {
      RCLCPP_WARN(get_logger(), "/get_planning_scene servisi yok.");
      return ObjInfo{};
    }

    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::msg::PlanningSceneComponents comp;
    comp.components = comp.WORLD_OBJECT_NAMES | comp.WORLD_OBJECT_GEOMETRY | comp.ALLOWED_COLLISION_MATRIX;
    req->components = comp;

    auto fut = cli->async_send_request(req);
    if (exec.spin_until_future_complete(fut, 4s) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "/get_planning_scene yanit vermedi.");
      return ObjInfo{};
    }
    auto resp = fut.get();
    if (!resp) return ObjInfo{};

    for (const auto &co : resp->scene.world.collision_objects) {
      if (co.id != id) continue;

      ObjInfo out;
      out.valid = true;
      out.id = co.id;

      if (!co.primitive_poses.empty())
        out.center = co.primitive_poses.front();
      else if (!co.mesh_poses.empty())
        out.center = co.mesh_poses.front();
      else
        out.center.orientation.w = 1.0;

      if (!co.primitives.empty() &&
          co.primitives.front().type == shape_msgs::msg::SolidPrimitive::BOX &&
          co.primitives.front().dimensions.size() >= 3)
      {
        out.size_x = co.primitives.front().dimensions[0];
        out.size_y = co.primitives.front().dimensions[1];
        out.size_z = co.primitives.front().dimensions[2];
      }
      return out;
    }
    return ObjInfo{};
  }

  ObjInfo queryObjectFromPSI(const std::string &id)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    auto objs = psi.getObjects({id});
    auto it = objs.find(id);
    if (it == objs.end()) return ObjInfo{};

    const auto &co = it->second;

    ObjInfo out;
    out.valid = true;
    out.id = id;

    if (!co.primitive_poses.empty())
      out.center = co.primitive_poses.front();
    else if (!co.mesh_poses.empty())
      out.center = co.mesh_poses.front();
    else
      out.center.orientation.w = 1.0;

    if (!co.primitives.empty() &&
        co.primitives.front().type == shape_msgs::msg::SolidPrimitive::BOX &&
        co.primitives.front().dimensions.size() >= 3)
    {
      out.size_x = co.primitives.front().dimensions[0];
      out.size_y = co.primitives.front().dimensions[1];
      out.size_z = co.primitives.front().dimensions[2];
    }
    return out;
  }

  ObjInfo serverDefaults(const std::string &id)
  {
    ObjInfo o;
    if (id == "box") {
      o.valid = true; o.id = "box";
      o.size_x = 0.04; o.size_y = 0.04; o.size_z = 0.12;
      o.center.orientation.w = 1.0;
      o.center.position.x = 0.50;
      o.center.position.y = 0.10;
      o.center.position.z = 0.46;
      return o;
    }
    if (id == "table") {
      o.valid = true; o.id = "table";
      o.size_x = 0.6; o.size_y = 0.5; o.size_z = 0.4;
      o.center.orientation.w = 1.0;
      o.center.position.x = 0.50;
      o.center.position.y = 0.00;
      o.center.position.z = 0.20;
      return o;
    }
    return ObjInfo{};
  }

  static bool isZeroPose(const geometry_msgs::msg::Pose &p)
  {
    return std::fabs(p.position.x) < 1e-6 &&
           std::fabs(p.position.y) < 1e-6 &&
           std::fabs(p.position.z) < 1e-6;
  }

  // ---- Tam POSE hedefiyle planla/çalıştır (aşağı bakan sabit oryantasyon) ----
  bool planAndExecutePose(moveit::planning_interface::MoveGroupInterface &mgi,
                          const geometry_msgs::msg::Pose &pose)
  {
    mgi.clearPoseTargets();
    mgi.setStartStateToCurrentState();
    mgi.setPoseTarget(pose, eef_link_);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok_plan = static_cast<bool>(mgi.plan(plan));
    if (!ok_plan) {
      RCLCPP_ERROR(get_logger(), "Planlama basarisiz (setPoseTarget).");
      return false;
    }
    auto ok_exec = static_cast<bool>(mgi.execute(plan));
    if (!ok_exec) {
      RCLCPP_ERROR(get_logger(), "Yurutme basarisiz.");
      return false;
    }
    return true;
  }

  // ---- Pre noktadan hedefe DÜZ-Z yönünde Cartesian iniş (başarısız olursa false) ----
  bool descendCartesian(moveit::planning_interface::MoveGroupInterface &mgi,
                        const geometry_msgs::msg::Pose &from,
                        const geometry_msgs::msg::Pose &to)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(from);
    waypoints.push_back(to);

    moveit_msgs::msg::RobotTrajectory traj;
    mgi.setStartStateToCurrentState();
    double frac = mgi.computeCartesianPath(waypoints, /*eef_step=*/0.01,
                                       traj, /*avoid_collisions=*/true);

    if (frac < 0.95) {
      RCLCPP_WARN(get_logger(), "Cartesian yol orani dusuk: %.2f", frac);
      return false;
    }
    auto ok_exec = static_cast<bool>(mgi.execute(traj));
    if (!ok_exec) {
      RCLCPP_ERROR(get_logger(), "Cartesian yurutme basarisiz.");
      return false;
    }
    return true;
  }

  void trySetNamed(moveit::planning_interface::MoveGroupInterface &mgi, const std::string &state)
  {
    const auto names = mgi.getNamedTargets();
    if (std::find(names.begin(), names.end(), state) == names.end()) return;
    try {
      mgi.setNamedTarget(state);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (static_cast<bool>(mgi.plan(plan)))
        (void)static_cast<bool>(mgi.execute(plan));
    } catch (...) {}
  }

  int fail(const char *msg)
  {
    RCLCPP_WARN(get_logger(), "%s", msg);
    RCLCPP_INFO(get_logger(), "Pick & Place BASARISIZ.");
    return 4;
  }

private:
  std::string planning_group_;
  std::string eef_link_;
  std::string world_frame_;

  double approach_distance_{0.18};
  double retreat_distance_{0.20};
  double place_dx_{-0.20};
  double place_dy_{-0.20};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PickPlaceNode>();
    int rc = node->run();
    rclcpp::shutdown();
    return rc;
  } catch (const std::exception &e) {
    fprintf(stderr, "FATAL: %s\n", e.what());
    rclcpp::shutdown();
    return 99;
  }
}
