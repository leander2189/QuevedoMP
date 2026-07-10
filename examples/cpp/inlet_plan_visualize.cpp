// examples/cpp/inlet_plan_visualize — a full Phase 3a pipeline run on the real DTC *inlet* cell:
// IK to the app's fastener pose → RRT-Connect plan → shortcut smooth → rerun animation.
//
// The rbrobout_inlet robot (UR10e on a 500 mm Ewellix lift, 7 DOF) starts at the app's
// initial_positions and must reach the "fastener_dock" target inside the inlet. The DTC app aims
// the TCP subframe `ee_9636_jA/ee_tcp_link` at the world `fastener_pose`; that subframe is a fixed
// offset (`tcp_pose`) from `tool_changer_robot_link` — the manipulator tip that IS in our URDF. So
// the IK target for the tip link is `fastener_pose · tcp_pose⁻¹`. The end-effector's own collision
// mesh is not in this flattened URDF, so (like dtc_visualize) collisions are robot-vs-work-object
// only (self-collision off) — exactly the "collision-free position inside the inlet" the app
// asserts.
//
//   Usage: inlet_plan_visualize [OUT_DIR=.]
//
// Build under a WITH_RERUN preset to get inlet_plan.rrd; with WITH_RERUN=OFF the Visualizer is a
// no-op and the program still runs + prints the IK/plan diagnostics (handy for CPU validation).
// See dtc_visualize.cpp for the Windows+Docker rerun/Arrow build-dir caveat.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/smoother.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

#include "dtc_scene.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

const Visualizer::Color kRobot{170, 175, 185};
const Visualizer::Color kGoalRobot{90, 200, 120};
const Visualizer::Color kEnv{200, 175, 120};
const Visualizer::Color kFastener{235, 70, 60};
const Visualizer::Color kTcpPath{70, 150, 235};

struct MeshPiece {
  int link;
  Mesh mesh;
  Transform origin;
};

std::vector<MeshPiece> load_robot_pieces(const RobotModel &model, const MeshSources &src) {
  std::vector<MeshPiece> out;
  const auto &links = model.links();
  for (int li = 0; li < static_cast<int>(links.size()); ++li)
    for (const CollisionGeometry &cg : links[li].collisions) {
      if (cg.type != GeometryType::Mesh)
        continue;
      Mesh m = load_mesh(resolve_mesh_uri(cg.mesh_filename, src.package_dirs, src.base_dir));
      for (Eigen::Vector3d &v : m.vertices)
        v = v.cwiseProduct(cg.mesh_scale);
      out.push_back({li, std::move(m), cg.origin});
    }
  return out;
}

// The 7-DOF start configuration from config/inlet/initial_positions.yaml (lift at the bottom; the
// six UR joints as given). Built by joint name so it is independent of dof-index ordering.
JointPosition start_config(const RobotModel &model) {
  JointPosition q = JointPosition::Zero(static_cast<Eigen::Index>(model.dof()));
  auto set = [&](const char *joint, double v) {
    if (const Joint *j = model.find_joint(joint); j && j->dof_index >= 0)
      q[j->dof_index] = v;
  };
  set("ewellix_lift_top_joint", 0.0);
  set("ur_shoulder_pan_joint", -1.0);
  set("ur_shoulder_lift_joint", -1.5);
  set("ur_elbow_joint", 0.0);
  set("ur_wrist_1_joint", -1.0);
  set("ur_wrist_2_joint", 0.0);
  set("ur_wrist_3_joint", -1.5708);
  return q;
}

// Densify a joint-space path so the rerun animation is smooth: insert interpolated configs so no
// step exceeds `step` on any joint. Endpoints preserved.
planning::Path densify(const planning::Path &path, double step) {
  planning::Path out;
  if (path.empty())
    return out;
  out.push_back(path.front());
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const JointPosition d = path[i + 1] - path[i];
    const int n = std::max(1, static_cast<int>(std::ceil(d.cwiseAbs().maxCoeff() / step)));
    for (int k = 1; k <= n; ++k)
      out.push_back(path[i] + (static_cast<double>(k) / n) * d);
  }
  return out;
}

} // namespace

int main(int argc, char **argv) {
  const std::string fx = QUEVEDOMP_FIXTURE_DIR;
  const std::string out_dir = argc > 1 ? argv[1] : ".";
  constexpr const char *kTip = "tool_changer_robot_link";

  // ---- Robot + inlet environment (the shared DTC builder) --------------------------------------
  const auto model = dtc::load_robot(fx, dtc::Scene::Inlet);
  const auto meshes = dtc::meshes(fx, dtc::Scene::Inlet);
  const SceneDescription env = dtc::make_env(fx, dtc::Scene::Inlet);
  auto robot = std::make_shared<RobotInstance>(model);
  dtc::load_acm(fx, robot->acm(), dtc::Scene::Inlet);

  const bool gpu = optix_available();
  std::shared_ptr<CollisionScene> scene = make_static_scene(
      model, env, gpu ? BackendHint::ForceOptix : BackendHint::ForceCpuFcl, meshes);

  QueryOptions opts;
  opts.check_self_collision = false; // robot-vs-inlet only (EE mesh absent from this URDF)

  // ---- Goal: aim the TCP subframe at the app's fastener pose -----------------------------------
  // The DTC app (stomp_seed_planner.cpp) places the EE's `ee_tcp_link` subframe at:
  //   TCP_world = tip_world · mesh_pose · tcp_pose      (tip = tool_changer_robot_link)
  // with mesh_pose / tcp_pose from config/inlet/eef_collision.yaml. So the IK target for the tip is
  //   tip_world = fastener · (mesh_pose · tcp_pose)⁻¹.
  const Transform fastener =
      dtc::pose_wxyz(-0.9427949, -0.2044888, 1.68938,                            // position
                     0.6936832, 0.7069808, -0.0134154, -0.1371198);              // w,x,y,z
  const Transform mesh_pose = dtc::pose_wxyz(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0); // w,x,y,z
  const Transform tcp_pose = dtc::pose_wxyz(0.0154998, 0.285, -0.2116998,        // position
                                            0.0, 0.0000003, 1.0, 0.0000003);     // w,x,y,z
  const Transform tcp_off = mesh_pose * tcp_pose; // tool_changer_robot_link → ee_tcp_link
  const Transform ik_target = fastener * tcp_off.inverse();

  const JointPosition start = start_config(*model);

  const auto ws = scene->make_workspace();
  auto collides = [&](const JointPosition &q) {
    return scene->query(*robot, q, opts, *ws).in_collision;
  };
  const bool start_free = !collides(start);
  std::printf("start collision-free: %s | tip@start=(%.3f,%.3f,%.3f) | fastener=(%.3f,%.3f,%.3f)\n",
              start_free ? "yes" : "NO", fk(*model, start, kTip).translation().x(),
              fk(*model, start, kTip).translation().y(), fk(*model, start, kTip).translation().z(),
              fastener.translation().x(), fastener.translation().y(), fastener.translation().z());

  // Find a collision-free IK solution for the tip reaching `tip_target` (7-DOF redundant ⇒ many
  // branches; seeded restarts diversify). Returns false if every branch that reaches the pose
  // collides with the inlet.
  auto find_free_ik = [&](const Transform &tip_target, JointPosition &out) {
    for (int s = 0; s < 160; ++s) {
      IkOptions io;
      io.max_restarts = 10;
      io.pos_tol = 1e-3;
      io.rot_tol = 1e-2;
      io.seed = 0x51ED0000ULL + static_cast<std::uint64_t>(s) * 0x9E3779B1ULL;
      const auto ik = make_numerical_ik(model, io);
      const IkResult ir = ik->solve(kTip, tip_target, s == 0 ? start : JointPosition());
      if (ir.success && !collides(ir.q)) {
        out = ir.q;
        return true;
      }
    }
    return false;
  };

  // Reaching a fastener deep in the inlet needs the app's slender jointA end-effector, which is NOT
  // in this flattened URDF — so the bare tool-changer arm can't dock exactly without the wrist
  // grazing the inlet. Back the TCP off along the approach axis (fastener → tool base) until a
  // collision-free "pre-dock" pose is found: the closest the bare arm can safely get to the
  // fastener.
  const Eigen::Vector3d shaft = (ik_target.translation() - fastener.translation()).normalized();
  JointPosition goal;
  double backoff = -1.0;
  const char *mode = nullptr;
  for (double d : {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6}) {
    const Transform backed = Transform::from_parts(fastener.translation() + d * shaft,
                                                   Eigen::Quaterniond(fastener.rotation()));
    if (find_free_ik(backed * tcp_off.inverse(), goal)) {
      backoff = d;
      mode = d == 0.0 ? "DOCKED at fastener (collision-free)" : "collision-free pre-dock";
      break;
    }
  }

  // Fallback: if no oriented pre-dock pose is free, take the collision-free sampled config whose
  // TCP is CLOSEST to the fastener (batch-first query over the pool). Guarantees a demoable goal
  // and quantifies how near the bare (EE-less) arm can get.
  if (backoff < 0.0) {
    Rng rng(11);
    const std::vector<JointPosition> pool = dtc::sample_configs(*model, rng, 6000);
    const BatchResult br = scene->query_batch(*robot, pool, opts, *ws);
    double best = 1e9;
    for (std::size_t i = 0; i < pool.size(); ++i) {
      if (br.in_collision[i])
        continue;
      const double dd =
          ((fk(*model, pool[i], kTip) * tcp_off).translation() - fastener.translation()).norm();
      if (dd < best) {
        best = dd;
        goal = pool[i];
      }
    }
    if (best > 1e8) {
      std::printf("  no collision-free configuration found at all; aborting.\n");
      return 2;
    }
    mode = "nearest collision-free approach (sampled)";
  }

  const Transform goal_tcp = fk(*model, goal, kTip) * tcp_off;
  std::printf("goal: %s | TCP=(%.3f,%.3f,%.3f) | %.3f m from fastener%s\n", mode,
              goal_tcp.translation().x(), goal_tcp.translation().y(), goal_tcp.translation().z(),
              (goal_tcp.translation() - fastener.translation()).norm(),
              backoff >= 0.0 ? "" : " (EE-less arm; app's slender jointA EE absent)");

  // ---- Plan (RRT-Connect) → smooth (shortcut) --------------------------------------------------
  std::shared_ptr<const RobotInstance> robot_c = robot;
  planning::PlannerParams pp;
  pp.edge_resolution = 0.05;
  pp.max_extension = 0.4;
  pp.batch_size = 64;
  const auto planner = planning::make_planner(pp, robot_c, scene);

  planning::PlanningProblem prob;
  prob.start = start;
  prob.goal = std::make_shared<planning::JointGoal>(goal, 1e-3);
  prob.collision = opts;
  prob.timeout = 60.0;
  prob.seed = 7;

  const planning::PlanningResult res = planner->plan(prob);
  std::printf("plan: %s (%s) | %zu waypoints | %llu collision queries (%llu configs) | %.2fs\n",
              planning::to_string(res.status), res.message.c_str(), res.path.size(),
              static_cast<unsigned long long>(res.stats.collision_queries),
              static_cast<unsigned long long>(res.stats.collision_configs), res.stats.time_total);
  if (!res.ok()) {
    std::printf("  no plan found; nothing to visualize.\n");
    return 3;
  }

  planning::SmootherParams sp;
  sp.edge_resolution = 0.05;
  sp.collision = opts;
  sp.seed = 7;
  const auto smoother = planning::make_shortcut_smoother(sp, robot_c, scene);
  const planning::Path smoothed = smoother->smooth(res.path);

  auto path_len = [](const planning::Path &p) {
    double L = 0;
    for (std::size_t i = 0; i + 1 < p.size(); ++i)
      L += (p[i + 1] - p[i]).norm();
    return L;
  };
  std::printf("smoothed: %zu -> %zu waypoints | length %.3f -> %.3f rad\n", res.path.size(),
              smoothed.size(), path_len(res.path), path_len(smoothed));

  // ---- Visualize (rerun) -----------------------------------------------------------------------
  Visualizer viz("quevedomp/inlet_plan");
  const std::string rrd = out_dir + "/inlet_plan.rrd";
  viz.save(rrd);

  // Static: the inlet work object, the fastener target point, and the TCP path polyline.
  for (const SceneObject &o : env.objects)
    if (const Mesh *m = std::get_if<Mesh>(&o.geometry))
      viz.log_mesh("world/env/" + o.id, *m, o.pose, kEnv, /*is_static=*/true);
  viz.log_points("world/fastener", {fastener.translation()}, kFastener, 0.03f);

  const planning::Path anim = densify(smoothed, 0.08);
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> tcp_segs;
  for (std::size_t i = 0; i + 1 < anim.size(); ++i)
    tcp_segs.emplace_back((fk(*model, anim[i], kTip) * tcp_off).translation(),
                          (fk(*model, anim[i + 1], kTip) * tcp_off).translation());
  viz.log_segments("world/tcp_path", tcp_segs, kTcpPath);

  const std::vector<MeshPiece> pieces = load_robot_pieces(*model, meshes);
  for (std::size_t k = 0; k < anim.size(); ++k) {
    viz.set_frame(static_cast<int>(k));
    const std::vector<Transform> tf = fk_all(*model, anim[k]);
    const Visualizer::Color color = (k + 1 == anim.size()) ? kGoalRobot : kRobot;
    for (std::size_t i = 0; i < pieces.size(); ++i)
      viz.log_mesh("world/robot/piece_" + std::to_string(i), pieces[i].mesh,
                   tf[pieces[i].link] * pieces[i].origin, color);
  }

  std::printf("inlet_plan_visualize: backend=%s | %zu animation frames -> %s\n",
              gpu ? "OptiX" : "FCL", anim.size(),
              viz.enabled() ? rrd.c_str() : "(viz disabled; no .rrd)");
  return 0;
}
