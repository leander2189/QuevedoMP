// examples/cpp/inlet_ik_visualize — a SIMPLE IK snapshot to SEE where the jointA end-effector
// collides with the inlet part. The rbrobout_inlet arm is IK'd to the app's fastener pose across
// several redundant branches; each branch is one rerun frame showing the arm (grey), the jointA EE
// collision mesh (green if it clears the inlet, red if it collides), the inlet part, and the
// fastener target. Scrub the frames to see the EE penetrate the duct.
//
// The EE mesh (jointA_closed_collision_v13.stl) is NOT part of the rbrobout_inlet URDF, so the arm
// collision scene never sees it. To get a real EE-vs-inlet answer we build a tiny one-link "EE
// probe" robot from that mesh and, per branch, move the inlet into the EE's frame and query it —
// exactly the check the DTC app does with the attached EE body.
//
//   Usage: inlet_ik_visualize [OUT_DIR=.]
//
// Build under a WITH_RERUN preset for inlet_ik.rrd; WITH_RERUN=OFF still prints per-branch
// diagnostics. See dtc_visualize.cpp for the Windows+Docker rerun/Arrow build-dir caveat.
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

#include "dtc_scene.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

const Visualizer::Color kArm{170, 175, 185};
const Visualizer::Color kEnv{200, 175, 120};
const Visualizer::Color kFastener{235, 70, 60};
const Visualizer::Color kEeFree{90, 200, 120};
const Visualizer::Color kEeHit{235, 70, 60};

struct MeshPiece {
  int link;
  Mesh mesh;
  Transform origin;
};

std::vector<MeshPiece> load_arm_pieces(const RobotModel &model, const MeshSources &src) {
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

} // namespace

int main(int argc, char **argv) {
  const std::string fx = QUEVEDOMP_FIXTURE_DIR;
  const std::string out_dir = argc > 1 ? argv[1] : ".";
  constexpr const char *kTip = "tool_changer_robot_link";
  const std::string ee_dir = fx + "/robots/meshes/dtc_test_inlet/meshes";

  // ---- Arm + inlet -----------------------------------------------------------------------------
  const auto model = dtc::load_robot(fx, dtc::Scene::Inlet);
  const auto meshes = dtc::meshes(fx, dtc::Scene::Inlet);
  const SceneDescription env = dtc::make_env(fx, dtc::Scene::Inlet);
  const RobotInstance robot(model);
  const auto arm_scene = make_static_scene(model, env, BackendHint::ForceCpuFcl, meshes);
  const auto arm_ws = arm_scene->make_workspace();
  QueryOptions opts;
  opts.check_self_collision = false;

  // ---- jointA EE probe: a one-link robot whose collision geometry IS the EE mesh ---------------
  // Used only to answer "does the EE overlap the inlet?" by moving the inlet into the EE frame.
  const Mesh ee_mesh = load_mesh(ee_dir + "/jointA_closed_collision_v13.stl");
  const char *ee_urdf = R"(<robot name="ee_probe"><link name="ee"><collision><geometry>
    <mesh filename="package://eep/jointA_closed_collision_v13.stl"/></geometry></collision></link></robot>)";
  const auto ee_model = RobotModel::from_urdf(ee_urdf);
  const RobotInstance ee_robot(ee_model);
  const MeshSources ee_src{{{"eep", ee_dir}}, ""};
  auto ee_scene = make_static_scene(ee_model, SceneDescription{}, BackendHint::ForceCpuFcl, ee_src);
  const Transform inlet_world = env.objects.front().pose;
  const SceneHandle inlet_h =
      ee_scene->add_object("inlet", env.objects.front().geometry, inlet_world);
  const auto ee_ws = ee_scene->make_workspace();

  // ---- Fastener TCP target (same composition as the DTC app; see inlet_plan_visualize) ---------
  const Transform fastener = dtc::pose_wxyz(-0.9427949, -0.2044888, 1.68938, //
                                            0.6936832, 0.7069808, -0.0134154, -0.1371198);
  const Transform mesh_pose = dtc::pose_wxyz(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  const Transform tcp_pose =
      dtc::pose_wxyz(0.0154998, 0.285, -0.2116998, 0.0, 0.0000003, 1.0, 0.0000003);
  const Transform tcp_off = mesh_pose * tcp_pose;
  const Transform ik_target = fastener * tcp_off.inverse();
  const JointPosition start = start_config(*model);

  // EE-vs-inlet by moving the inlet into the EE frame (EE probe sits at its own origin).
  auto ee_hits_inlet = [&](const JointPosition &q) {
    const Transform ee_world = fk(*model, q, kTip) * mesh_pose;
    ee_scene->move_object(inlet_h, ee_world.inverse() * inlet_world);
    return ee_scene->query(ee_robot, JointPosition(), opts, *ee_ws).in_collision;
  };

  // ---- Collect IK branches that reach the fastener --------------------------------------------
  struct Branch {
    JointPosition q;
    double pos_err;
    bool arm_hit;
    bool ee_hit;
  };
  std::vector<Branch> branches;
  for (int s = 0; s < 40 && branches.size() < 12; ++s) {
    IkOptions io;
    io.max_restarts = 12;
    io.pos_tol = 1e-3;
    io.rot_tol = 1e-2;
    io.seed = 0x1CE00000ULL + static_cast<std::uint64_t>(s) * 0x9E3779B1ULL;
    const auto ik = make_numerical_ik(model, io);
    const IkResult r = ik->solve(kTip, ik_target, s == 0 ? start : JointPosition());
    if (!r.success)
      continue;
    branches.push_back({r.q, r.pos_error, arm_scene->query(robot, r.q, opts, *arm_ws).in_collision,
                        ee_hits_inlet(r.q)});
  }
  std::printf("IK branches reaching the fastener: %zu\n", branches.size());
  if (branches.empty()) {
    std::printf("  none reached the fastener pose; aborting.\n");
    return 1;
  }
  for (std::size_t k = 0; k < branches.size(); ++k)
    std::printf("  branch %2zu: TCP err %.4g m | arm-vs-inlet %s | EE-vs-inlet %s\n", k,
                branches[k].pos_err, branches[k].arm_hit ? "HIT" : "free",
                branches[k].ee_hit ? "HIT" : "free");

  // ---- Visualize -------------------------------------------------------------------------------
  Visualizer viz("quevedomp/inlet_ik");
  const std::string rrd = out_dir + "/inlet_ik.rrd";
  viz.save(rrd);
  for (const SceneObject &o : env.objects)
    if (const Mesh *m = std::get_if<Mesh>(&o.geometry))
      viz.log_mesh("world/env/" + o.id, *m, o.pose, kEnv, /*is_static=*/true);

  const std::vector<MeshPiece> pieces = load_arm_pieces(*model, meshes);
  for (std::size_t k = 0; k < branches.size(); ++k) {
    viz.set_frame(static_cast<int>(k));
    const std::vector<Transform> tf = fk_all(*model, branches[k].q);
    for (std::size_t i = 0; i < pieces.size(); ++i)
      viz.log_mesh("world/arm/piece_" + std::to_string(i), pieces[i].mesh,
                   tf[pieces[i].link] * pieces[i].origin, kArm);
    // The EE mesh at its real pose, tinted by whether it overlaps the inlet.
    const Transform ee_world = fk(*model, branches[k].q, kTip) * mesh_pose;
    viz.log_mesh("world/ee", ee_mesh, ee_world, branches[k].ee_hit ? kEeHit : kEeFree);
    viz.log_points("world/fastener", {fastener.translation()}, kFastener, 0.03f);
  }

  std::printf("inlet_ik_visualize: %zu branches -> %s\n", branches.size(),
              viz.enabled() ? rrd.c_str() : "(viz disabled; no .rrd)");
  return 0;
}
