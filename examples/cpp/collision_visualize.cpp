// examples/cpp/collision_visualize — SEE collision queries: a robot swept toward an obstacle,
// with the colliding link tinted red and the signed-distance witness (nearest/deepest pair) drawn
// as a point-pair + segment (Task 2a.7). Built for eyeballing the FCL result now and, in Phase 2b,
// for spotting where OptiX and FCL disagree.
//
//   Usage: collision_visualize [FIXTURES_DIR=tests/fixtures] [OUT_DIR=.]
//
// Build with the dev-viz preset (WITH_RERUN=ON) to get collision.rrd; with WITH_RERUN=OFF it still
// builds and runs (the Visualizer is a no-op) but writes nothing.
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

const Visualizer::Color kGray{180, 180, 190};
const Visualizer::Color kRed{220, 40, 40};
const Visualizer::Color kYellow{240, 220, 40};
const Visualizer::Color kBlue{80, 120, 220};

std::string read_file(const std::string &path) {
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Closed axis-aligned box mesh (half-extents), centered at origin — to render a box obstacle.
Mesh box_mesh(const Eigen::Vector3d &h) {
  Mesh m;
  for (int i = 0; i < 8; ++i)
    m.vertices.emplace_back(i & 1 ? h.x() : -h.x(), i & 2 ? h.y() : -h.y(), i & 4 ? h.z() : -h.z());
  const int f[12][3] = {{0, 1, 3}, {0, 3, 2}, {4, 6, 7}, {4, 7, 5}, {0, 4, 5}, {0, 5, 1},
                        {2, 3, 7}, {2, 7, 6}, {0, 2, 6}, {0, 6, 4}, {1, 5, 7}, {1, 7, 3}};
  for (const auto &t : f)
    m.triangles.emplace_back(t[0], t[1], t[2]);
  return m;
}

// Pose + log the robot's collision meshes by FK at q; the link named `hot` is tinted red.
void log_robot_meshes(Visualizer &viz, const std::string &root, const RobotModel &model,
                      const JointPosition &q,
                      const std::unordered_map<std::string, std::string> &packages,
                      const std::string &hot) {
  const std::vector<Transform> tf = fk_all(model, q);
  const auto &links = model.links();
  for (std::size_t i = 0; i < links.size(); ++i) {
    int c = 0;
    for (const auto &col : links[i].collisions) {
      if (col.type != GeometryType::Mesh)
        continue;
      Mesh m = load_mesh(resolve_mesh_uri(col.mesh_filename, packages, ""));
      for (auto &v : m.vertices)
        v = v.cwiseProduct(col.mesh_scale);
      const auto color = links[i].name == hot ? kRed : kGray;
      viz.log_mesh(root + "/mesh/" + links[i].name + "_" + std::to_string(c++), m,
                   tf[i] * col.origin, color);
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  const std::string fixtures = argc > 1 ? argv[1] : "tests/fixtures";
  const std::string out_dir = argc > 2 ? argv[2] : ".";
  const std::string mesh_root = fixtures + "/robots/meshes";
  const std::unordered_map<std::string, std::string> packages = {
      {"example-robot-data", mesh_root + "/example-robot-data"}};

  const auto model = RobotModel::from_urdf(read_file(fixtures + "/robots/ur5.urdf"));
  const RobotInstance robot(model);
  const int dof = static_cast<int>(model->dof());
  const std::string tip = "wrist_3_link";

  // Sweep from home to a folded config; drop a sphere obstacle where the wrist ends up, so the arm
  // drives into it partway through, plus a static floor box for context.
  const JointPosition home = JointPosition::Zero(dof);
  const JointPosition folded = JointPosition::Constant(dof, -0.8);
  const Eigen::Vector3d ball_c = fk(*model, folded, tip).translation();

  SceneDescription env;
  env.objects.push_back({"ball", SphereShape{0.15}, Transform::from_translation(ball_c)});
  env.objects.push_back({"floor", BoxShape{Eigen::Vector3d(0.6, 0.6, 0.02)},
                         Transform::from_translation(Eigen::Vector3d(0, 0, -0.15))});
  const auto scene = make_static_scene(model, env, BackendHint::Auto, MeshSources{packages, ""});
  const auto ws = scene->make_workspace();

  Visualizer viz("quevedomp/collision");
  const std::string rrd = out_dir + "/collision.rrd";
  viz.save(rrd);

  // Static environment (logged once): the sphere as a point-with-radius, the floor as a mesh.
  viz.log_points("world/env/ball", {ball_c}, kBlue, 0.15f);
  viz.log_mesh("world/env/floor", box_mesh(Eigen::Vector3d(0.6, 0.6, 0.02)), env.objects[1].pose,
               kGray);

  QueryOptions opts;
  opts.distance = true;
  opts.max_distance = 1.0f;
  opts.check_self_collision = false; // isolate robot-vs-environment for the demo

  const int frames = 40;
  int n_collide = 0;
  for (int k = 0; k <= frames; ++k) {
    const double t = static_cast<double>(k) / frames;
    const JointPosition q = home + t * (folded - home);
    const CollisionResult r = scene->query(robot, q, opts, *ws);

    viz.set_frame(k);
    const std::string hot = (r.in_collision && r.witness) ? r.witness->a : std::string();
    log_robot_meshes(viz, "world/robot", *model, q, packages, hot);

    if (r.witness) {
      viz.log_points("world/witness/pts", {r.witness->point_a, r.witness->point_b}, kYellow, 0.02f);
      viz.log_segments("world/witness/seg", {{r.witness->point_a, r.witness->point_b}}, kYellow);
    }
    if (r.in_collision)
      ++n_collide;
  }

  std::cout << "collision_visualize: swept " << (frames + 1) << " configs, " << n_collide
            << " colliding -> " << (viz.enabled() ? rrd : std::string("(viz disabled; no .rrd)"))
            << "\n";
  return 0;
}
