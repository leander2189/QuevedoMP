// benchmarks/bench_collision — FCL (CPU) vs OptiX (GPU) collision-query latency/throughput.
//
// This is the Task 2b.2 "small-batch latency profile": for a real mesh robot (UR5) vs a static box
// environment, it times query_batch on both backends across a range of batch sizes and reports
// per-batch latency, throughput, and the GPU/CPU speedup. Build it under the bench-optix preset
// (Release, sanitizers OFF) — Debug + ASan/UBSan would slow the CPU (FCL) path enough to make the
// comparison meaningless.
//
// Methodology: scenes/workspaces are built ONCE (context + pipeline + GAS + FK-ray precompute are
// one-time setup, not per-query), then each backend runs a warm-up batch (lazy CUDA init, first
// buffer growth) before the timed repetitions — so we measure the steady-state RRT hot path, not
// first-call overhead. Robot-vs-self is off by default to isolate the GPU robot-vs-environment path
// that OptiX actually accelerates (a second pass turns it on to show full-query cost).
//
//   Usage: bench_collision                 # the synthetic triangle-count sweep (default)
//          bench_collision <mesh> [scale]  # one REAL mesh from disk as the environment
//
// The second form is the "does the synthetic wall predict a real high-poly part?" check: it runs the
// low-poly box baseline (to pin this machine and this session) and then the loaded mesh, so the
// file's numbers sit in the same table as the procedural sweep. `scale` is a uniform factor applied
// to the vertices, because STL carries no unit metadata: a millimetre-authored file needs 0.001
// (load_mesh only honors units for the formats that record them, e.g. COLLADA).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <span>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/core/rng.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/kinematics/fk.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

#include "bench_harness.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

std::string fixtures() { return std::string(QUEVEDOMP_FIXTURE_DIR); }

std::string read_text(const std::string &path) {
  std::ifstream f(path);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

MeshSources ur5_meshes() {
  return MeshSources{{{"example-robot-data", fixtures() + "/robots/meshes/example-robot-data"}}, ""};
}

// Random configs uniformly inside the joint limits (continuous joints fall back to [-pi, pi]).
std::vector<JointPosition> sample_configs(const RobotModel &m, Rng &rng, int n) {
  const int dof = static_cast<int>(m.dof());
  Eigen::VectorXd lo(dof), hi(dof);
  for (const Joint &j : m.joints()) {
    if (!j.is_movable())
      continue;
    const int i = j.dof_index;
    if (j.limits.has_position_limit) {
      lo[i] = j.limits.lower;
      hi[i] = j.limits.upper;
    } else {
      lo[i] = -M_PI;
      hi[i] = M_PI;
    }
  }
  std::vector<JointPosition> qs;
  qs.reserve(n);
  for (int k = 0; k < n; ++k)
    qs.push_back(rng.sample_in_box(lo, hi));
  return qs;
}

// A small static box environment scattered around the UR5 workspace, tuned so random configs give a
// MIX of collisions and frees (an all-hit or all-miss scene would bias the early-out behavior). The
// LOW-poly baseline: 4 boxes = 48 triangles, which barely exercises either backend's BVH.
SceneDescription make_box_env() {
  SceneDescription env;
  auto box = [&](const char *id, Eigen::Vector3d he, Eigen::Vector3d at) {
    env.objects.push_back({id, BoxShape{he}, Transform::from_translation(at)});
  };
  box("wall_x", {0.05, 0.6, 0.6}, {0.55, 0.0, 0.4});
  box("wall_y", {0.6, 0.05, 0.6}, {0.0, 0.55, 0.4});
  box("pillar", {0.08, 0.08, 0.8}, {-0.4, -0.3, 0.4});
  box("shelf", {0.4, 0.4, 0.04}, {0.0, 0.0, 0.85});
  return env;
}

// A tessellated wall (regular grid) in the plane x = px, spanning [y0,y1] x [z0,z1], subdivided into
// n x n cells => 2*n*n triangles. This is the HIGH-poly lever: identical Mesh geometry feeds the
// OptiX triangle GAS and the FCL BVH, so densifying it stresses exactly the acceleration structures
// the two backends differ on. Placed inside the arm's reach so random configs cross it (mix of
// hit/free). It is an open surface (not watertight) — fine for a surface-crossing benchmark.
Mesh make_wall_mesh(double px, double y0, double y1, double z0, double z1, int n) {
  Mesh m;
  m.vertices.reserve(static_cast<std::size_t>(n + 1) * (n + 1));
  for (int i = 0; i <= n; ++i)
    for (int j = 0; j <= n; ++j)
      m.vertices.emplace_back(px, y0 + (y1 - y0) * i / n, z0 + (z1 - z0) * j / n);
  auto idx = [&](int i, int j) { return i * (n + 1) + j; };
  m.triangles.reserve(static_cast<std::size_t>(2) * n * n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      m.triangles.emplace_back(idx(i, j), idx(i + 1, j), idx(i + 1, j + 1));
      m.triangles.emplace_back(idx(i, j), idx(i + 1, j + 1), idx(i, j + 1));
    }
  return m;
}

// A dense-mesh environment of ~target_tris triangles: a wall the UR5 sweeps into. Spans the whole
// workspace, so nearly every link is near it (adversarial for the broadphase cull).
SceneDescription make_mesh_env(int target_tris) {
  const int n = std::max(1, static_cast<int>(std::lround(std::sqrt(target_tris / 2.0))));
  SceneDescription env;
  env.objects.push_back(
      {"wall", make_wall_mesh(0.35, -0.6, 0.6, 0.0, 1.0, n), Transform::Identity()});
  return env;
}

// The environment as ONE mesh loaded from disk, uniformly scaled. Unlike the procedural wall this
// is whatever geometry the file holds — arbitrary topology, uneven triangle density, its own world
// placement — which is the point: it is the reality check on the synthetic sweep. The bounding box is
// printed because placement is what sets the collision fraction, and a mesh authored in the wrong
// units (STL records none) then shows up here as a box a thousand times too big, instead of as a
// mysteriously all-free benchmark.
SceneDescription make_file_mesh_env(const std::string &path, double scale) {
  Mesh m = load_mesh(path);
  if (scale != 1.0)
    for (Eigen::Vector3d &v : m.vertices)
      v *= scale;

  Eigen::Vector3d lo = Eigen::Vector3d::Constant(1e30), hi = Eigen::Vector3d::Constant(-1e30);
  for (const Eigen::Vector3d &v : m.vertices) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  std::printf("mesh %s: %zu triangles, %zu vertices, scale %g\n", path.c_str(),
              m.triangles.size(), m.vertices.size(), scale);
  std::printf("  bbox min [%.3f %.3f %.3f] max [%.3f %.3f %.3f] size [%.3f %.3f %.3f] (m)\n",
              lo.x(), lo.y(), lo.z(), hi.x(), hi.y(), hi.z(), hi.x() - lo.x(), hi.y() - lo.y(),
              hi.z() - lo.z());

  SceneDescription env;
  env.objects.push_back({"file_mesh", std::move(m), Transform::Identity()});
  return env;
}

// Same triangle budget, but a small (0.3x0.3 m) dense panel off in one corner that only the extended
// arm reaches — so the base/shoulder/upper links stay far from it. This is where a robot-link
// broadphase cull can skip most links' rays; contrast its A/B with make_mesh_env.
SceneDescription make_localized_mesh_env(int target_tris) {
  const int n = std::max(1, static_cast<int>(std::lround(std::sqrt(target_tris / 2.0))));
  SceneDescription env;
  env.objects.push_back(
      {"panel", make_wall_mesh(0.5, 0.25, 0.55, 0.55, 0.85, n), Transform::Identity()});
  return env;
}

using bench::fraction_true;
using bench::run_table;
using bench::time_ms;

// Build both backends over `env`, sanity-check they agree (outside the ±1e-4 band), and run the
// env-only latency table. Reports the environment triangle count and collision fraction.
void bench_env(const char *label, const std::shared_ptr<const RobotModel> &model,
               const RobotInstance &robot, const SceneDescription &env,
               const std::vector<JointPosition> &pool, const std::vector<int> &batches,
               int rep_budget) {
  std::size_t env_tris = 0;
  for (const SceneObject &o : env.objects)
    if (const auto *m = std::get_if<Mesh>(&o.geometry))
      env_tris += m->triangles.size();
    else
      env_tris += 12; // a box tessellates to 12 triangles in the OptiX GAS

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, ur5_meshes());
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, ur5_meshes());

  QueryOptions opts;
  opts.check_self_collision = false; // isolate the GPU robot-vs-environment path

  auto fws = fcl->make_workspace();
  auto ows = optix->make_workspace();
  const std::span<const JointPosition> probe(pool.data(), std::min<std::size_t>(2000, pool.size()));
  const BatchResult f = fcl->query_batch(robot, probe, opts, *fws);
  const BatchResult o = optix->query_batch(robot, probe, opts, *ows);
  std::size_t disagree = 0;
  for (std::size_t i = 0; i < probe.size(); ++i)
    disagree += (f.in_collision[i] != o.in_collision[i]) ? 1 : 0;

  std::printf("\n########## %s: env = %zu triangles ##########\n", label, env_tris);
  std::printf("agreement: %zu/%zu disagree | collision fraction FCL=%.2f OptiX=%.2f\n", disagree,
              probe.size(), fraction_true(f.in_collision), fraction_true(o.in_collision));
  run_table("robot-vs-environment only", *fcl, *optix, robot, pool, opts, batches, rep_budget);
}

} // namespace

int main(int argc, char **argv) {
  if (!optix_available()) {
    std::printf("OptiX backend not built — configure with the bench-optix preset.\n");
    return 1;
  }

  const std::string mesh_path = argc > 1 ? argv[1] : "";
  const double mesh_scale = argc > 2 ? std::atof(argv[2]) : 1.0;

  const auto model = RobotModel::from_urdf(read_text(fixtures() + "/robots/ur5.urdf"));
  const RobotInstance robot(model);

  Rng rng(12345);
  const std::vector<JointPosition> pool = sample_configs(*model, rng, 10000);
  std::printf("robot: UR5 (mesh links, ~5.3k tris)   pool: %zu random configs\n", pool.size());

  // Both backends compute FK on the host per config (fk_all) before any collision work. Time it in
  // isolation: it is the shared floor both query paths sit on top of, and — as the mesh sweep shows —
  // the dominant cost of the whole query at these batch sizes.
  {
    volatile std::size_t sink = 0;
    const int reps = 20;
    const double ms = time_ms(reps, [&] {
      for (const JointPosition &q : pool)
        sink += fk_all(*model, q).size();
    });
    std::printf("host FK floor (fk_all over %zu configs): %.3f ms total, %.3f us/config\n",
                pool.size(), ms, ms * 1e3 / pool.size());
    (void)sink;
  }

  // A mesh on the command line replaces the sweep: the box baseline pins this machine and session,
  // then the real file gets the same table. Batch 10000 is in the list here — a multi-million-triangle
  // part is exactly the fat-batch regime the sweep's 500k wall was standing in for.
  if (!mesh_path.empty()) {
    bench_env("LOW-POLY BASELINE (boxes)", model, robot, make_box_env(), pool, {1, 10, 100, 1000},
              20000);
    bench_env("FILE MESH", model, robot, make_file_mesh_env(mesh_path, mesh_scale), pool,
              {1, 10, 100, 1000, 10000}, 20000);
    return 0;
  }

  std::printf("Sweeping ENVIRONMENT triangle count — the acceleration-structure lever that "
              "separates the GPU BVH from FCL.\n");

  // Low-poly baseline (boxes) then a dense-mesh sweep. Small batches use more repetitions; the big
  // env x large batch combos are capped by rep_budget so the run stays a few minutes.
  bench_env("LOW-POLY BASELINE (boxes)", model, robot, make_box_env(), pool, {1, 10, 100, 1000},
            20000);
  // The ~50-tri mesh isolates the host-bound floor (near-zero GPU traversal, no containment since it
  // is non-watertight): whatever it costs is FK + transform assembly + transfers, not the BVH.
  for (int tris : {50, 5000, 50000, 500000})
    bench_env("HIGH-POLY MESH (spanning wall)", model, robot, make_mesh_env(tris), pool,
              {1, 10, 100, 1000}, 20000);

  // Localized obstacle — where the opt-in broadphase cull (QUEVEDOMP_OPTIX_CULL) should pay off.
  bench_env("LOCALIZED MESH (corner panel)", model, robot, make_localized_mesh_env(50000), pool,
            {1, 10, 100, 1000}, 20000);

  return 0;
}
