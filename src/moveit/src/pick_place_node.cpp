#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <control_msgs/action/gripper_command.hpp>

using namespace std::chrono_literals;

class PickPlaceNode : public rclcpp::Node
{
public:
  using GripperCmd = control_msgs::action::GripperCommand;
  using GripperClient = rclcpp_action::Client<GripperCmd>;

  PickPlaceNode()
  : Node("pick_place_node",
         rclcpp::NodeOptions()
           .allow_undeclared_parameters(true)
           .automatically_declare_parameters_from_overrides(true))
  {
    planning_group_ = pget<std::string>("planning_group", "panda_arm");
    eef_link_       = pget<std::string>("eef_link",       "panda_link8");
    world_frame_    = pget<std::string>("world_frame",    "panda_link0"); // server ile ayni

    vel_scale_      = pget<double>("vel_scale", 0.35);
    acc_scale_      = pget<double>("acc_scale", 0.35);
    eef_step_       = pget<double>("eef_step",  0.004);
    cart_min_frac_  = pget<double>("cart_min_frac", 0.45);
    cart_segments_  = pget<int>("cart_segments", 6);

    approach_z_     = pget<double>("approach_z", 0.24);
    retreat_z_      = pget<double>("retreat_z",  0.20);
    pick_clearance_ = pget<double>("pick_clearance", 0.08);
    place_dx_       = pget<double>("place_dx",  -0.20);
    place_dy_       = pget<double>("place_dy",  -0.20);

    ensure_box_     = pget<bool>("ensure_box", true);
    allow_touch_    = pget<bool>("allow_touch", true);
    attach_after_pick_ = pget<bool>("attach_after_pick", true);

    box_x_ = pget<double>("box_x", 0.50);
    box_y_ = pget<double>("box_y", 0.10);
    box_z_ = pget<double>("box_z", 0.445);
    box_sx_ = pget<double>("box_size_x", 0.06);
    box_sy_ = pget<double>("box_size_y", 0.06);
    box_sz_ = pget<double>("box_size_z", 0.05);

    // Gripper
    gripper_action_ns_ = pget<std::string>("gripper_action", "/panda_hand_controller/gripper_cmd");
    grip_open_  = pget<double>("grip_open",  0.080);
    grip_close_ = pget<double>("grip_close", 0.030);
    grip_effort_= pget<double>("grip_effort", 40.0);
    grip_wait_s_= pget<double>("grip_wait_s", 3.0);

    pensure<std::string>("robot_description", "");
    pensure<std::string>("robot_description_semantic", "");

    gripper_client_ = rclcpp_action::create_client<GripperCmd>(this, gripper_action_ns_);

    RCLCPP_INFO(get_logger(), "PickPlaceNode hazir. group=%s eef=%s frame=%s",
                planning_group_.c_str(), eef_link_.c_str(), world_frame_.c_str());
  }

  int run()
  {
    rclcpp::executors::SingleThreadedExecutor exec;
    auto self = this->shared_from_this();
    exec.add_node(self);

    if (!mirror_moveit_params(exec)) {
      RCLCPP_FATAL(get_logger(), "FATAL: MoveIt parametreleri mirror edilemedi.");
      return 2;
    }

    moveit::planning_interface::MoveGroupInterface mgi(self, planning_group_);
    mgi.setPoseReferenceFrame(world_frame_);
    mgi.setEndEffectorLink(eef_link_);
    mgi.setPlannerId("RRTConnectkConfigDefault");
    mgi.setPlanningTime(10.0);
    mgi.setNumPlanningAttempts(20);
    mgi.setMaxVelocityScalingFactor(vel_scale_);
    mgi.setMaxAccelerationScalingFactor(acc_scale_);
    mgi.setGoalPositionTolerance(0.01);
    mgi.setGoalOrientationTolerance(0.7);
    mgi.setGoalJointTolerance(0.01);
    mgi.setWorkspace(-1.0, -1.0, 0.0, 1.5, 1.5, 1.5);

    rclcpp::sleep_for(300ms);

    if (ensure_box_) ensure_box_in_scene();

    ObjInfo box = query_object_robust("box", exec);
    if (!box.valid) {
      RCLCPP_FATAL(get_logger(), "Box bilgisi bulunamadi (sahne/psi/default).");
      return 3;
    }

    const double half_h = (box.size_z > 1e-6 ? box.size_z * 0.5 : 0.06);
    const double table_top_z = 0.40;
    const double box_top_z   = box.center.position.z + half_h;
    const double place_cx = box.center.position.x + place_dx_;
    const double place_cy = box.center.position.y + place_dy_;

    auto q_down = [](){ geometry_msgs::msg::Quaternion q; q.w=0.0; q.x=1.0; q.y=0.0; q.z=0.0; return q; }();
    auto pose_at = [&](double x,double y,double z){
      geometry_msgs::msg::Pose p; p.position.x=x; p.position.y=y; p.position.z=z; p.orientation=q_down; return p;
    };

    const geometry_msgs::msg::Pose pre_pick  = pose_at(box.center.position.x, box.center.position.y,
                                                       box_top_z + approach_z_);
    const geometry_msgs::msg::Pose pick      = pose_at(box.center.position.x, box.center.position.y,
                                                       box_top_z + pick_clearance_);

    const geometry_msgs::msg::Pose pre_place = pose_at(place_cx, place_cy,
                                                       table_top_z + half_h + approach_z_);
    const geometry_msgs::msg::Pose place     = pose_at(place_cx, place_cy,
                                                       table_top_z + half_h + pick_clearance_);

    // 1) yaklaşmadan önce gripper açık
    (void)gripper_open(exec);

    if (!plan_and_execute_pose(mgi, pre_pick,  "pre-pick", /*relaxed*/true))
      return fail_and_exit("Plan bulunamadi (pre-pick).");

    // 2) dik iniş (segmentli kartesyen) ya da fallback plan
    if (!descend_cartesian_segmented(mgi, pre_pick, pick))
      if (!plan_and_execute_pose(mgi, pick, "pick", /*relaxed*/true))
        return fail_and_exit("Plan bulunamadi (pick).");

    // 3) gripper kapat ve ataşla
    (void)gripper_close(exec);
    if (attach_after_pick_) {
      attach_object_to_eef("box");
      RCLCPP_INFO(get_logger(), "Obje EEF'e attach edildi.");
    }

    // 4) yukarı kaçış
    geometry_msgs::msg::Pose retreat = pick;
    retreat.position.z += retreat_z_;
    (void)descend_cartesian_segmented(mgi, pick, retreat);
    rclcpp::sleep_for(200ms);

    // 5) pre-place ve place
    if (!plan_and_execute_pose(mgi, pre_place, "pre-place", /*relaxed*/true))
      return fail_and_exit("Plan bulunamadi (pre-place).");

    if (!descend_cartesian_segmented(mgi, pre_place, place))
      if (!plan_and_execute_pose(mgi, place, "place", /*relaxed*/true))
        return fail_and_exit("Plan bulunamadi (place).");

    // 6) brak ve detach
    (void)gripper_open(exec);
    detach_object_from_eef("box");

    // 7) yukarı kalk
    geometry_msgs::msg::Pose post_place = place;
    post_place.position.z += retreat_z_;
    (void)descend_cartesian_segmented(mgi, place, post_place);

    RCLCPP_INFO(get_logger(), "Pick & Place BASARILI.");
    return 0;
  }

private:
  // ---------- param helpers ----------
  template<typename T>
  T pget(const std::string &name, const T &def) {
    if (!this->has_parameter(name)) this->declare_parameter<T>(name, def);
    return this->get_parameter(name).get_value<T>();
  }
  template<typename T>
  void pensure(const std::string &name, const T &def) {
    if (!this->has_parameter(name)) this->declare_parameter<T>(name, def);
  }

  // ---------- mirror (URDF/SRDF + prefixes) ----------
  bool mirror_moveit_params(rclcpp::executors::SingleThreadedExecutor &exec)
  {
    std::string urdf, srdf, who_urdf, who_srdf;
    const std::vector<std::string> prefer_urdf = {"/move_group","move_group", "/robot_state_publisher","robot_state_publisher", "/rviz2","rviz2"};
    const std::vector<std::string> prefer_srdf = {"/move_group","move_group", "/rviz2","rviz2"};

    auto deadline = this->now() + rclcpp::Duration(30,0);

    while (this->now() < deadline) { if (discover_param_string(exec, prefer_urdf, "robot_description", urdf, who_urdf)) break; rclcpp::sleep_for(500ms); }
    while (this->now() < deadline) { if (discover_param_string(exec, prefer_srdf, "robot_description_semantic", srdf, who_srdf)) break; rclcpp::sleep_for(500ms); }

    if (urdf.empty() || srdf.empty()) return false;
    RCLCPP_INFO(get_logger(), "[mirror] URDF from %s, SRDF from %s", who_urdf.c_str(), who_srdf.c_str());

    this->set_parameters({ rclcpp::Parameter("robot_description", urdf),
                           rclcpp::Parameter("robot_description_semantic", srdf) });

    size_t kcopied = mirror_prefix(exec, "/move_group", "robot_description_kinematics");
    size_t pcopied = mirror_prefix(exec, "/move_group", "robot_description_planning");
    if (pcopied == 0) RCLCPP_WARN(get_logger(), "[mirror] move_group'tan planning paramlari gelmedi (kritik degil).");
    RCLCPP_INFO(get_logger(), "[mirror] kinematics=%zu, planning=%zu kopyalandi.", kcopied, pcopied);
    return true;
  }

  size_t mirror_prefix(rclcpp::executors::SingleThreadedExecutor &exec,
                       const std::string &remote_node, const std::string &prefix)
  {
    auto cli = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this(), remote_node);
    if (!cli->wait_for_service(3s)) return 0;

    auto fut_list = cli->list_parameters({prefix}, 1000);
    if (exec.spin_until_future_complete(fut_list, 3s) != rclcpp::FutureReturnCode::SUCCESS) return 0;
    auto listed = fut_list.get();
    if (listed.names.empty()) return 0;

    size_t copied = 0;
    const size_t chunk = 40;
    for (size_t i=0; i<listed.names.size(); i+=chunk) {
      std::vector<std::string> part(listed.names.begin()+i, listed.names.begin()+std::min(listed.names.size(), i+chunk));
      auto fut_get = cli->get_parameters(part);
      if (exec.spin_until_future_complete(fut_get, 4s) != rclcpp::FutureReturnCode::SUCCESS) continue;
      auto vals = fut_get.get();
      if (vals.empty()) {
        RCLCPP_WARN(get_logger(), "[mirror] get_parameters bos dondu (chunk %zu..%zu).", i, i+part.size());
        continue;
      }
      this->set_parameters(vals);
      copied += vals.size();
    }
    return copied;
  }

  bool discover_param_string(rclcpp::executors::SingleThreadedExecutor &exec,
                             const std::vector<std::string> &nodes,
                             const std::string &key, std::string &out, std::string &who)
  {
    for (int pass=0; pass<5; ++pass) {
      for (const auto &n : nodes)
        if (try_get_param_string(exec, n, key, out)) { who = n; return true; }
      rclcpp::sleep_for(500ms);
    }
    return false;
  }

  bool try_get_param_string(rclcpp::executors::SingleThreadedExecutor &exec,
                            const std::string &remote_node, const std::string &key, std::string &out)
  {
    auto cli = std::make_shared<rclcpp::AsyncParametersClient>(this->shared_from_this(), remote_node);
    if (!cli->wait_for_service(3s)) return false;
    auto fut = cli->get_parameters({key});
    if (exec.spin_until_future_complete(fut, 3s) != rclcpp::FutureReturnCode::SUCCESS) return false;
    auto res = fut.get();
    if (res.empty() || res[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) return false;
    out = res[0].as_string();
    return !out.empty();
  }

  // ---------- scene: box ----------
  void ensure_box_in_scene()
  {
    static const std::string kBoxId = "box";
    moveit::planning_interface::PlanningSceneInterface psi;

    auto objs = psi.getObjects({kBoxId});
    if (objs.find(kBoxId) != objs.end()) {
      RCLCPP_INFO(get_logger(), "Kutu sahnede zaten var; yeniden eklenmeyecek.");
      return;
    }

    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::BOX;
    prim.dimensions = {box_sx_, box_sy_, box_sz_};

    geometry_msgs::msg::Pose p; p.orientation.w = 1.0;
    p.position.x = box_x_; p.position.y = box_y_; p.position.z = box_z_;

    moveit_msgs::msg::CollisionObject co;
    co.id = kBoxId; co.header.frame_id = world_frame_;
    co.primitives.push_back(prim); co.primitive_poses.push_back(p);
    co.operation = co.ADD;

    psi.applyCollisionObject(co);
    RCLCPP_INFO(get_logger(), "Kutu sahneye eklendi (%.3f, %.3f, %.3f, %.3fx%.3fx%.3f).",
                box_x_, box_y_, box_z_, box_sx_, box_sy_, box_sz_);
    rclcpp::sleep_for(200ms);
  }

  // ---------- object query ----------
  struct ObjInfo {
    bool valid{false};
    std::string id;
    geometry_msgs::msg::Pose center{};
    double size_x{0}, size_y{0}, size_z{0};
    std::string source;
  };

  ObjInfo query_object_robust(const std::string &id, rclcpp::executors::SingleThreadedExecutor &exec)
  {
    for (int i=1; i<=8; ++i) {
      ObjInfo s = query_from_scene(id, exec);
      if (s.valid && !is_zero_pose(s.center)) { s.source="scene"; return s; }
      if (s.valid && is_zero_pose(s.center))
        RCLCPP_WARN(get_logger(), "Obj '%s' sahnede ama poz (0,0,0) gorunuyor (try %d/8).", id.c_str(), i);
      rclcpp::sleep_for(300ms);
    }
    ObjInfo p = query_from_psi(id); if (p.valid && !is_zero_pose(p.center)) { p.source="psi"; return p; }
    ObjInfo d = defaults_local(id); if (d.valid) { d.source="default"; return d; }
    return ObjInfo{};
  }

  ObjInfo query_from_scene(const std::string &id, rclcpp::executors::SingleThreadedExecutor &exec)
  {
    auto cli = this->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!cli->wait_for_service(2s)) return ObjInfo{};

    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    moveit_msgs::msg::PlanningSceneComponents comp;
    comp.components = comp.WORLD_OBJECT_NAMES | comp.WORLD_OBJECT_GEOMETRY;
    req->components = comp;

    auto fut = cli->async_send_request(req);
    if (exec.spin_until_future_complete(fut, 3s) != rclcpp::FutureReturnCode::SUCCESS) return ObjInfo{};
    auto resp = fut.get(); if (!resp) return ObjInfo{};

    for (const auto &co : resp->scene.world.collision_objects) {
      if (co.id != id) continue;
      ObjInfo out; out.valid = true; out.id = id;
      if (!co.primitive_poses.empty()) out.center = co.primitive_poses.front();
      else if (!co.mesh_poses.empty()) out.center = co.mesh_poses.front();
      else out.center.orientation.w = 1.0;

      if (!co.primitives.empty() &&
          co.primitives.front().type == shape_msgs::msg::SolidPrimitive::BOX &&
          co.primitives.front().dimensions.size() >= 3) {
        out.size_x = co.primitives.front().dimensions[0];
        out.size_y = co.primitives.front().dimensions[1];
        out.size_z = co.primitives.front().dimensions[2];
      }
      return out;
    }
    return ObjInfo{};
  }

  ObjInfo query_from_psi(const std::string &id)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    auto objs = psi.getObjects({id});
    auto it = objs.find(id); if (it == objs.end()) return ObjInfo{};
    const auto &co = it->second;

    ObjInfo out; out.valid = true; out.id = id;
    if (!co.primitive_poses.empty()) out.center = co.primitive_poses.front();
    else if (!co.mesh_poses.empty()) out.center = co.mesh_poses.front();
    else out.center.orientation.w = 1.0;

    if (!co.primitives.empty() &&
        co.primitives.front().type == shape_msgs::msg::SolidPrimitive::BOX &&
        co.primitives.front().dimensions.size() >= 3) {
      out.size_x = co.primitives.front().dimensions[0];
      out.size_y = co.primitives.front().dimensions[1];
      out.size_z = co.primitives.front().dimensions[2];
    }
    return out;
  }

  ObjInfo defaults_local(const std::string &id)
  {
    ObjInfo o;
    if (id == "box") {
      o.valid = true; o.id = "box";
      o.size_x = (box_sx_>0?box_sx_:0.04);
      o.size_y = (box_sy_>0?box_sy_:0.04);
      o.size_z = (box_sz_>0?box_sz_:0.12);
      o.center.orientation.w = 1.0;
      o.center.position.x = box_x_;
      o.center.position.y = box_y_;
      o.center.position.z = box_z_;
      return o;
    }
    return ObjInfo{};
  }

  static bool is_zero_pose(const geometry_msgs::msg::Pose &p)
  {
    return std::fabs(p.position.x) < 1e-6 &&
           std::fabs(p.position.y) < 1e-6 &&
           std::fabs(p.position.z) < 1e-6;
  }

  // ---------- timing helper ----------
  static void add_uniform_timestamps(moveit_msgs::msg::RobotTrajectory &traj, double dt_sec = 0.02)
  {
    if (traj.joint_trajectory.points.empty()) return;
    double t = 0.0;
    for (auto &pt : traj.joint_trajectory.points) {
      int64_t nsec = static_cast<int64_t>(t * 1e9 + 0.5);
      pt.time_from_start.sec = static_cast<int32_t>(nsec / 1000000000LL);
      pt.time_from_start.nanosec = static_cast<uint32_t>(nsec % 1000000000LL);
      t += dt_sec;
    }
  }

  // ---------- planning helpers ----------
  bool plan_and_execute_pose(moveit::planning_interface::MoveGroupInterface &mgi,
                             const geometry_msgs::msg::Pose &pose,
                             const char* tag,
                             bool relaxed)
  {
    mgi.clearPoseTargets();
    rclcpp::sleep_for(50ms);
    mgi.setStartStateToCurrentState();
    mgi.setPoseTarget(pose, eef_link_);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mgi.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      if (mgi.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS) return true;
      RCLCPP_ERROR(get_logger(), "Yurutme basarisiz (%s).", tag);
      return false;
    }
    if (!relaxed) { RCLCPP_ERROR(get_logger(), "Plan bulunamadi (%s).", tag); return false; }

    try {
      if (mgi.setApproximateJointValueTarget(pose, eef_link_)) {
        moveit::planning_interface::MoveGroupInterface::Plan p2;
        if (mgi.plan(p2) == moveit::core::MoveItErrorCode::SUCCESS &&
            mgi.execute(p2) == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(get_logger(), "[%s] setApproximateJointValueTarget ile ulasildi.", tag);
          return true;
        }
      }
    } catch (...) {}

    mgi.clearPoseTargets();
    mgi.setStartStateToCurrentState();
    mgi.setPositionTarget(pose.position.x, pose.position.y, pose.position.z, eef_link_);
    moveit::planning_interface::MoveGroupInterface::Plan p3;
    if (mgi.plan(p3) == moveit::core::MoveItErrorCode::SUCCESS &&
        mgi.execute(p3) == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "[%s] yalniz pozisyon hedefi ile ulasildi.", tag);
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Plan bulunamadi (%s) [tum fallback'ler bitti].", tag);
    return false;
  }

  // düz-iniş: segmentli
  bool descend_cartesian_segmented(moveit::planning_interface::MoveGroupInterface &mgi,
                                   const geometry_msgs::msg::Pose &from,
                                   const geometry_msgs::msg::Pose &to)
  {
    auto cart_one = [&](const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b)->bool{
      std::vector<geometry_msgs::msg::Pose> wps; wps.reserve(2); wps.push_back(a); wps.push_back(b);
      moveit_msgs::msg::RobotTrajectory traj;
      mgi.setStartStateToCurrentState();
      double frac = mgi.computeCartesianPath(wps, eef_step_, traj, /*avoid_collisions=*/true);
      if (frac < cart_min_frac_) {
        RCLCPP_WARN(get_logger(), "Segment kartesyen frac=%.2f (min %.2f) -> red.", frac, cart_min_frac_);
        return false;
      }
      add_uniform_timestamps(traj, 0.02);  // 20 ms adım
      auto rc = mgi.execute(traj);
      if (rc != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Kartesyen yurutme basarisiz.");
        return false;
      }
      rclcpp::sleep_for(80ms);
      return true;
    };

    geometry_msgs::msg::Pose prev = from;
    const int N = std::max(1,cart_segments_);
    for (int i=1; i<=N; ++i) {
      const double t = static_cast<double>(i) / N;
      geometry_msgs::msg::Pose mid = to;
      mid.position.x = from.position.x + t*(to.position.x - from.position.x);
      mid.position.y = from.position.y + t*(to.position.y - from.position.y);
      mid.position.z = from.position.z + t*(to.position.z - from.position.z);
      mid.orientation = to.orientation;

      if (!cart_one(prev, mid)) {
        RCLCPP_WARN(get_logger(), "Segment %d/%d kartesyen yol uretilemedi (min_frac=%.2f).", i, N, cart_min_frac_);
        return false;
      }
      prev = mid;
    }
    return true;
  }

  // ---------- gripper helpers (AYNI EXECUTOR İLE!) ----------
  bool gripper_open (rclcpp::executors::SingleThreadedExecutor& exec) { return send_gripper_command(exec, grip_open_,  grip_effort_, grip_wait_s_, "open"); }
  bool gripper_close(rclcpp::executors::SingleThreadedExecutor& exec) { return send_gripper_command(exec, grip_close_, grip_effort_, grip_wait_s_, "close"); }

  bool send_gripper_command(rclcpp::executors::SingleThreadedExecutor& exec,
                            double width, double effort, double wait_s, const char* tag)
  {
    if (!gripper_client_->wait_for_action_server(1s)) {
      RCLCPP_WARN(get_logger(), "[gripper] Action server yok (%s). Devam.", gripper_action_ns_.c_str());
      return false;
    }
    auto goal = GripperCmd::Goal();
    goal.command.position   = width;
    goal.command.max_effort = effort;

    auto send_opts = typename GripperClient::SendGoalOptions();
    send_opts.result_callback = [](const rclcpp_action::ClientGoalHandle<GripperCmd>::WrappedResult&){};

    auto fut_goal = gripper_client_->async_send_goal(goal, send_opts);
    if (exec.spin_until_future_complete(fut_goal, std::chrono::duration<double>(wait_s))
          != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(get_logger(), "[gripper] goal gönderilemedi (%s).", tag);
      return false;
    }
    auto ghandle = fut_goal.get();
    if (!ghandle) {
      RCLCPP_WARN(get_logger(), "[gripper] goal handle null (%s).", tag);
      return false;
    }
    auto fut_res = gripper_client_->async_get_result(ghandle);
    (void)exec.spin_until_future_complete(fut_res, std::chrono::duration<double>(wait_s));
    RCLCPP_INFO(get_logger(), "[gripper] komut tamam (%s, hedef=%.3f).", tag, width);
    return true;
  }

  // ---------- attach/detach ----------
  void attach_object_to_eef(const std::string &object_id)
  {
    moveit::planning_interface::PlanningSceneInterface psi;
    std::vector<std::string> touch_links = {
      "panda_hand", "panda_leftfinger", "panda_rightfinger", eef_link_
    };
    if (!allow_touch_) touch_links.clear();

    try {
      moveit::planning_interface::MoveGroupInterface mgi(this->shared_from_this(), planning_group_);
      mgi.attachObject(object_id, /*link=*/eef_link_, touch_links);
    } catch (...) {
      moveit_msgs::msg::AttachedCollisionObject aco;
      aco.object.id = object_id;
      aco.link_name = eef_link_;
      aco.touch_links = touch_links;
      aco.object.operation = aco.object.ADD;
      psi.applyAttachedCollisionObject(aco);
    }
    rclcpp::sleep_for(100ms);
  }

  void detach_object_from_eef(const std::string &object_id)
  {
    try {
      moveit::planning_interface::MoveGroupInterface mgi(this->shared_from_this(), planning_group_);
      mgi.detachObject(object_id);
    } catch (...) {
      moveit::planning_interface::PlanningSceneInterface psi;
      moveit_msgs::msg::AttachedCollisionObject aco;
      aco.object.id = object_id;
      aco.link_name = eef_link_;
      aco.object.operation = aco.object.REMOVE;
      psi.applyAttachedCollisionObject(aco);
    }
    rclcpp::sleep_for(100ms);
  }

  int fail_and_exit(const char *msg)
  {
    RCLCPP_ERROR(get_logger(), "%s", msg);
    RCLCPP_INFO(get_logger(), "BASARISIZ.");
    return 4;
  }

private:
  // params
  std::string planning_group_, eef_link_, world_frame_;
  double vel_scale_{0.35}, acc_scale_{0.35}, eef_step_{0.004}, cart_min_frac_{0.45};
  int cart_segments_{6};
  double approach_z_{0.24}, retreat_z_{0.20}, pick_clearance_{0.08}, place_dx_{-0.20}, place_dy_{-0.20};
  bool ensure_box_{true}, allow_touch_{true}, attach_after_pick_{true};
  double box_x_{0.50}, box_y_{0.10}, box_z_{0.445}, box_sx_{0.06}, box_sy_{0.06}, box_sz_{0.05};

  // gripper
  std::string gripper_action_ns_;
  double grip_open_{0.080}, grip_close_{0.030}, grip_effort_{40.0}, grip_wait_s_{3.0};
  GripperClient::SharedPtr gripper_client_;
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
