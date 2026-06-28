// examples/cpp/visualize — produce a rerun .rrd per robot so you can SEE FK/IK working
// (Task 1.8). Build with the dev-viz preset (WITH_RERUN=ON); open the output with the matching
// rerun viewer. With WITH_RERUN=OFF this still builds but does nothing (Visualizer is a no-op).
//
//   Usage: visualize [FIXTURES_DIR=tests/fixtures] [OUT_DIR=.]
//
// For each robot it logs: the posed robot (link frames + skeleton + collision meshes via FK),
// an IK target frame vs. the achieved end-effector frame, and the tip path of a straight-line
// joint interpolation toward the IK solution.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/kinematics/ik.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

namespace {

using namespace quevedomp;

std::string read_file(const std::string &path) {
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

struct RobotCfg {
  const char *urdf;
  const char *base_subdir; // for relative mesh paths (iiwa); "" otherwise
  std::string tip;
};

// Pose the robot's collision meshes (resolved + loaded) by FK at configuration q.
void log_robot_meshes(Visualizer &viz, const std::string &root, const RobotModel &model,
                      const JointPosition &q,
                      const std::unordered_map<std::string, std::string> &packages,
                      const std::string &base_dir) {
  const std::vector<Transform> tf = fk_all(model, q);
  const auto &links = model.links();
  for (std::size_t i = 0; i < links.size(); ++i) {
    int c = 0;
    for (const auto &col : links[i].collisions) {
      if (col.type != GeometryType::Mesh)
        continue;
      try {
        const std::string path = resolve_mesh_uri(col.mesh_filename, packages, base_dir);
        Mesh m = load_mesh(path);
        for (auto &v : m.vertices)
          v = v.cwiseProduct(col.mesh_scale); // apply URDF <mesh scale>
        viz.log_mesh(root + "/mesh/" + links[i].name + "_" + std::to_string(c++), m,
                     tf[i] * col.origin);
      } catch (const std::exception &e) {
        std::cerr << "  (skip mesh " << col.mesh_filename << ": " << e.what() << ")\n";
      }
    }
  }
}

void visualize_one(const std::string &fixtures, const std::string &out_dir, const RobotCfg &cfg,
                   const std::unordered_map<std::string, std::string> &packages) {
  const std::string name = std::string(cfg.urdf).substr(0, std::string(cfg.urdf).find('.'));
  const auto model = RobotModel::from_urdf(read_file(fixtures + "/robots/" + cfg.urdf));
  const std::string base_dir = std::string(cfg.base_subdir).empty()
                                   ? std::string()
                                   : fixtures + "/robots/meshes/" + cfg.base_subdir;

  Visualizer viz("quevedomp/" + name);
  const std::string rrd = out_dir + "/" + name + ".rrd";
  viz.save(rrd);

  const int dof = static_cast<int>(model->dof());
  const JointPosition home = JointPosition::Zero(dof);

  // Posed robot (frames + skeleton + meshes) at a representative config.
  JointPosition q = JointPosition::Constant(dof, 0.4);
  viz.log_robot("world/" + name, *model, q);
  log_robot_meshes(viz, "world/" + name, *model, q, packages, base_dir);

  // IK demo: target = FK at another config; solve from home and show target vs. achieved frames.
  const Transform target = fk(*model, JointPosition::Constant(dof, -0.3), cfg.tip);
  const auto ik = make_numerical_ik(model);
  const auto res = ik->solve(cfg.tip, target, home);
  viz.log_pose("world/" + name + "/ik_target", target);
  if (res.success)
    viz.log_pose("world/" + name + "/ik_achieved", fk(*model, res.q, cfg.tip));

  // Tip path of a straight-line joint interpolation home -> IK solution.
  Trajectory traj;
  const JointPosition goal = res.success ? res.q : q;
  for (int s = 0; s <= 30; ++s) {
    Waypoint wp;
    wp.time = s / 30.0;
    wp.state.pos = home + (goal - home) * (s / 30.0);
    traj.push_back(wp);
  }
  viz.log_trajectory("world/" + name + "/tip_path", *model, traj, cfg.tip);

  std::cout << "  " << name << ": dof=" << dof << ", ik=" << (res.success ? "ok" : "FAILED")
            << " -> " << (viz.enabled() ? rrd : std::string("(viz disabled; no .rrd)")) << "\n";
}

} // namespace

int main(int argc, char **argv) {
  const std::string fixtures = (argc > 1) ? argv[1] : "tests/fixtures";
  const std::string out_dir = (argc > 2) ? argv[2] : ".";

  const std::string mesh_root = fixtures + "/robots/meshes";
  const std::unordered_map<std::string, std::string> packages = {
      {"example-robot-data", mesh_root + "/example-robot-data"},
      {"collision", mesh_root + "/abb_irb2400/collision"}};

  const std::vector<RobotCfg> robots = {{"ur5.urdf", "", "wrist_3_link"},
                                        {"ur10.urdf", "", "wrist_3_link"},
                                        {"panda.urdf", "", "panda_link8"},
                                        {"iiwa.urdf", "kuka_iiwa", ""},
                                        {"irb2400.urdf", "", "link_6"}};

  std::cout << "Writing rerun recordings to " << out_dir << " ...\n";
  for (RobotCfg cfg : robots) {
    if (cfg.tip.empty()) { // default tip = last link
      const auto model =
          quevedomp::RobotModel::from_urdf(read_file(fixtures + "/robots/" + cfg.urdf));
      cfg.tip = model->links().back().name;
    }
    try {
      visualize_one(fixtures, out_dir, cfg, packages);
    } catch (const std::exception &e) {
      std::cerr << "  " << cfg.urdf << ": ERROR " << e.what() << "\n";
    }
  }
  return 0;
}
