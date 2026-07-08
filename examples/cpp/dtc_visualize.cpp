// examples/cpp/dtc_visualize — SEE the OptiX collision result on the real DTC cell. The rbrobout
// robot (UR10e on an Ewellix lift + the ee_hilok end-effector) is sampled against the work-object
// environment; each pose is classified by the OptiX boolean and the whole robot is tinted red
// (collision) or grey (free). The work object + fiducial markers are drawn once for context.
//
//   Usage: dtc_visualize [OUT_DIR=.] [mt_part|inlet]   (default scene mt_part)
//
// Build under the viz-optix preset (WITH_RERUN=ON + WITH_OPTIX=ON) to get dtc_collision.rrd, then
// open it in the rerun viewer. With WITH_RERUN=OFF the Visualizer is a no-op and nothing is written;
// with WITH_OPTIX=OFF it still runs but classifies with FCL (optix_available() == false).
//
// NOTE (Windows + Docker/drvfs): the rerun SDK builds Arrow, whose CheckSymbolExists try_compile
// fails ("could not be removed") when the CMake build dir lives on the /mnt bind mount. Put the
// build dir on the container-local filesystem instead — source stays on the mount:
//   cmake -S /workspace -B /tmp/vizbuild -G Ninja -DCMAKE_BUILD_TYPE=Release \
//     -DQUEVEDOMP_WITH_CUDA=ON -DQUEVEDOMP_WITH_OPTIX=ON -DQUEVEDOMP_WITH_RERUN=ON \
//     -DQUEVEDOMP_ENABLE_SANITIZERS=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5
//   cmake --build /tmp/vizbuild --target dtc_visualize && /tmp/vizbuild/examples/cpp/dtc_visualize OUT_DIR
// On native Linux the viz-optix preset builds in place with no workaround.
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"
#include "quevedomp/viz/visualizer.hpp"

#include "dtc_scene.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

const Visualizer::Color kGrey{170, 175, 185};
const Visualizer::Color kRed{225, 55, 55};
const Visualizer::Color kEnv{200, 175, 120};
const Visualizer::Color kMarker{110, 140, 210};

// One posed robot collision-mesh piece, loaded once (its link, geometry, and link-frame origin).
struct MeshPiece {
  int link;
  Mesh mesh;
  Transform origin;
};

// Load every robot collision MESH once (primitive links — the mobile base boxes / tool-changer
// cylinder — are skipped; the arm, lift and EE are all meshes, which is what we want to see).
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

} // namespace

int main(int argc, char **argv) {
  const std::string fx = QUEVEDOMP_FIXTURE_DIR;
  const std::string out_dir = argc > 1 ? argv[1] : ".";
  const bool inlet = argc > 2 && std::string(argv[2]) == "inlet";
  const dtc::Scene sel = inlet ? dtc::Scene::Inlet : dtc::Scene::MtPart;

  const auto model = dtc::load_robot(fx, sel);
  const RobotInstance robot(model);
  const auto meshes = dtc::meshes(fx, sel);
  const SceneDescription env = dtc::make_env(fx, sel);

  const bool gpu = optix_available();
  const auto scene = make_static_scene(model, env, gpu ? BackendHint::ForceOptix : BackendHint::ForceCpuFcl, meshes);
  const auto ws = scene->make_workspace();
  const std::vector<MeshPiece> pieces = load_robot_pieces(*model, meshes);

  // Classify a large sample with the (OptiX) backend, then pick a balanced set of colliding and free
  // poses so the animation shows plenty of both despite the scene's low collision rate.
  Rng rng(5);
  const std::vector<JointPosition> pool = dtc::sample_configs(*model, rng, 4000);
  QueryOptions opts;
  opts.check_self_collision = false; // robot-vs-work-object — the boolean OptiX computes
  const BatchResult r = scene->query_batch(robot, pool, opts, *ws);

  std::vector<std::size_t> collide, free;
  for (std::size_t i = 0; i < pool.size(); ++i)
    (r.in_collision[i] ? collide : free).push_back(i);

  std::vector<std::size_t> frames;
  for (std::size_t i = 0; i < collide.size() && i < 25; ++i)
    frames.push_back(collide[i]);
  for (std::size_t i = 0; i < free.size() && i < 25; ++i)
    frames.push_back(free[i]);

  Visualizer viz("quevedomp/dtc");
  const std::string rrd = out_dir + (inlet ? "/dtc_collision_inlet.rrd" : "/dtc_collision_mt_part.rrd");
  viz.save(rrd);

  // Static environment (logged once, on rerun's static timeline so it shows at every frame): the
  // work object as a mesh, markers in a distinct colour.
  for (const SceneObject &o : env.objects)
    if (const Mesh *m = std::get_if<Mesh>(&o.geometry))
      viz.log_mesh("world/env/" + o.id, *m, o.pose, o.id == "work_object" ? kEnv : kMarker,
                   /*is_static=*/true);

  for (std::size_t k = 0; k < frames.size(); ++k) {
    const JointPosition &q = pool[frames[k]];
    const bool hit = r.in_collision[frames[k]] != 0;
    viz.set_frame(static_cast<int>(k));
    const std::vector<Transform> tf = fk_all(*model, q);
    const Visualizer::Color color = hit ? kRed : kGrey;
    for (std::size_t i = 0; i < pieces.size(); ++i)
      viz.log_mesh("world/robot/piece_" + std::to_string(i), pieces[i].mesh,
                   tf[pieces[i].link] * pieces[i].origin, color);
  }

  std::printf("dtc_visualize: backend=%s | %zu colliding + %zu free of %zu poses | %zu frames -> %s\n",
              gpu ? "OptiX" : "FCL", collide.size(), free.size(), pool.size(), frames.size(),
              viz.enabled() ? rrd.c_str() : "(viz disabled; no .rrd)");
  return 0;
}
