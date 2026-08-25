// benchmarks/bench_dtc — FCL vs OptiX on the real DTC cell: the rbrobout robot (UR10e on an Ewellix
// 900 mm lift, with the ee_hilok end-effector) against the work-object environment (mesh.stl + the
// fiducial markers) at their app world poses. Random 7-DOF poses, batch-size latency/throughput
// sweep — the same harness as bench_collision, on a real industrial scene instead of a synthetic
// wall.
//
// The work object is a LOCALIZED obstacle far from the base (only the extended arm + EE reach it), so
// this is the regime the opt-in robot-link broadphase cull targets. A/B it by running the binary
// twice: once plain, once with QUEVEDOMP_OPTIX_CULL=1. Build under the bench-optix preset.
//
//   Usage: bench_dtc [mt_part|inlet]   (default mt_part)
//
// Beyond the two latency tables there is a COST DECOMPOSITION pass that answers "where does a full
// query's time actually go?" by timing the same batch under four option/scene combinations —
// floor (no env, self off), env-only, self-only, full — so the robot-vs-self and robot-vs-env
// contributions can be read off separately instead of inferred from the full-query total.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/core/mesh_io.hpp"
#include "quevedomp/robot/mesh_resolver.hpp"
#include "quevedomp/robot/robot_instance.hpp"

#include "bench_harness.hpp"
#include "dtc_scene.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

// One timing point: best per-config microseconds (harness minimum-of-trials) plus the collision
// fraction that pass observed. The fraction belongs next to the number because every path here
// early-outs on the first overlap — a pass that collides more does less work per config.
struct Point {
  double us = 0.0;
  double frac = 0.0;
};

Point time_point(const CollisionScene &scene, const RobotInstance &robot,
                 const std::vector<JointPosition> &pool, int batch, const QueryOptions &opts,
                 int rep_budget) {
  auto ws = scene.make_workspace();
  const std::span<const JointPosition> qs(pool.data(), static_cast<std::size_t>(batch));
  const BatchResult warm = scene.query_batch(robot, qs, opts, *ws); // warm-up + collision fraction
  const int reps = std::max(3, rep_budget / batch);
  const double ms = bench::time_ms(reps, [&] { scene.query_batch(robot, qs, opts, *ws); });
  return {ms * 1e3 / batch, bench::fraction_true(warm.in_collision)};
}

// Time the four combinations on one backend and print the derived per-component costs.
//
//   A floor = empty env, self off  -> FK + link-transform fill + broadphase update (shared floor)
//   B env   = real  env, self off  -> floor + robot-vs-environment
//   C self  = empty env, self on   -> floor + robot-vs-self
//   D full  = real  env, self on   -> the query the planner actually issues
//
// so env = B - A, self = C - A, and A + (B-A) + (C-A) is what D would cost if the two phases were
// independent and additive. D coming in UNDER that is the early-out (env-colliding configs never
// reach the self pass) and, on the OptiX backend, the CPU/GPU overlap.
void decompose(const char *backend, const CollisionScene &bare, const CollisionScene &full_scene,
               const RobotInstance &robot, const std::vector<JointPosition> &pool, int batch,
               int rep_budget) {
  QueryOptions off, on;
  off.check_self_collision = false;
  on.check_self_collision = true;

  const Point a = time_point(bare, robot, pool, batch, off, rep_budget);
  const Point b = time_point(full_scene, robot, pool, batch, off, rep_budget);
  const Point c = time_point(bare, robot, pool, batch, on, rep_budget);
  const Point d = time_point(full_scene, robot, pool, batch, on, rep_budget);

  std::printf("\n-- %s, batch %d (us per config; [] = collision fraction) --\n", backend, batch);
  std::printf("  A floor  (no env, self off) : %8.3f  [%.3f]\n", a.us, a.frac);
  std::printf("  B env    (env,    self off) : %8.3f  [%.3f]\n", b.us, b.frac);
  std::printf("  C self   (no env, self on)  : %8.3f  [%.3f]\n", c.us, c.frac);
  std::printf("  D full   (env,    self on)  : %8.3f  [%.3f]\n", d.us, d.frac);
  const double env = b.us - a.us, self = c.us - a.us;
  std::printf("  => env  = B-A = %7.3f us  (%4.1f%% of D)\n", env, 100.0 * env / d.us);
  std::printf("  => self = C-A = %7.3f us  (%4.1f%% of D)   self/env = %.2fx\n", self,
              100.0 * self / d.us, env > 0.0 ? self / env : 0.0);
  std::printf("  => floor(FK+xform+broadphase) = %7.3f us  (%4.1f%% of D)\n", a.us,
              100.0 * a.us / d.us);
  std::printf("  additivity: A+env+self = %7.3f us vs measured D = %7.3f us  (D/sum = %.2f)\n",
              a.us + env + self, d.us, d.us / (a.us + env + self));
}

// Structural anatomy behind the numbers: how many link shapes the self pass broadphases against
// each other, how many pairs the ACM removes, and how much triangle budget sits on each side.
void print_anatomy(const RobotModel &model, const RobotInstance &robot,
                   const SceneDescription &env, const MeshSources &meshes) {
  std::size_t shapes = 0, robot_tris = 0, prims = 0;
  for (const Link &l : model.links())
    for (const CollisionGeometry &cg : l.collisions) {
      ++shapes;
      if (cg.type != GeometryType::Mesh) {
        ++prims;
        continue;
      }
      try {
        robot_tris +=
            load_mesh(resolve_mesh_uri(cg.mesh_filename, meshes.package_dirs, meshes.base_dir))
                .triangles.size();
      } catch (const std::exception &) { // counting only — a miss must not kill the benchmark
      }
    }

  // Unordered link-shape pairs, minus same-link pairs, minus the ones the SRDF ACM allows.
  std::size_t pairs = shapes * (shapes - 1) / 2, allowed = 0;
  const auto &links = model.links();
  std::vector<int> shape_link;
  shape_link.reserve(shapes);
  for (std::size_t li = 0; li < links.size(); ++li)
    for (std::size_t k = 0; k < links[li].collisions.size(); ++k)
      shape_link.push_back(static_cast<int>(li));
  for (std::size_t i = 0; i < shape_link.size(); ++i)
    for (std::size_t j = i + 1; j < shape_link.size(); ++j)
      if (shape_link[i] == shape_link[j] ||
          robot.acm().is_allowed(links[shape_link[i]].name, links[shape_link[j]].name))
        ++allowed;

  std::size_t env_tris = 0;
  for (const SceneObject &o : env.objects)
    if (const auto *m = std::get_if<Mesh>(&o.geometry))
      env_tris += m->triangles.size();

  std::printf("anatomy: %zu robot collision shapes (%zu primitives, %zu triangles total) | "
              "self pairs %zu total - %zu allowed/same-link = %zu CHECKED\n",
              shapes, prims, robot_tris, pairs, allowed, pairs - allowed);
  std::printf("         environment: %zu object(s), %zu triangles | ACM entries: %zu\n",
              env.objects.size(), env_tris, robot.acm().size());
}

} // namespace

int main(int argc, char **argv) {
  if (!optix_available()) {
    std::printf("OptiX backend not built — configure with the bench-optix preset.\n");
    return 1;
  }
  const std::string fx = QUEVEDOMP_FIXTURE_DIR;
  const bool inlet = argc > 1 && std::string(argv[1]) == "inlet";
  const dtc::Scene scene = inlet ? dtc::Scene::Inlet : dtc::Scene::MtPart;

  const auto model = dtc::load_robot(fx, scene);
  RobotInstance robot(model);
  dtc::load_acm(fx, robot.acm(), scene); // SRDF allowed-collision matrix (for the full-query pass)
  const auto meshes = dtc::meshes(fx, scene);
  const SceneDescription env = dtc::make_env(fx, scene);

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, meshes);
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, meshes);

  Rng rng(2024);
  const std::vector<JointPosition> pool = dtc::sample_configs(*model, rng, 10000);

  const bool cull = std::getenv("QUEVEDOMP_OPTIX_CULL") != nullptr;
  std::printf("DTC cell: %s (dof=%zu) vs work object (%zu env meshes)\n",
              inlet ? "inlet / bmt_9636 (UR10e + 500mm lift + dress-kits + jointA EE)"
                    : "mt_part / rbrobout (UR10e + 900mm lift + ee_hilok)",
              model->dof(), env.objects.size());
  std::printf("OptiX robot-link broadphase cull: %s\n",
              cull ? "ON (QUEVEDOMP_OPTIX_CULL set)" : "off (default)");

  // Sanity: the backends must agree on this real scene, and it must give a genuine collision mix.
  {
    auto fws = fcl->make_workspace();
    auto ows = optix->make_workspace();
    const std::span<const JointPosition> qs(pool.data(), 2000);
    QueryOptions o;
    o.check_self_collision = false;
    const BatchResult f = fcl->query_batch(robot, qs, o, *fws);
    const BatchResult g = optix->query_batch(robot, qs, o, *ows);
    std::size_t disagree = 0;
    for (std::size_t i = 0; i < qs.size(); ++i)
      disagree += (f.in_collision[i] != g.in_collision[i]) ? 1 : 0;
    std::printf("agreement (2000 poses): %zu disagree | collision fraction FCL=%.3f OptiX=%.3f\n",
                disagree, bench::fraction_true(f.in_collision), bench::fraction_true(g.in_collision));
  }

  QueryOptions env_only;
  env_only.check_self_collision = false;
  bench::run_table("DTC robot-vs-work-object (self off — isolates the GPU path)", *fcl, *optix, robot,
                   pool, env_only, {1, 10, 100, 1000, 10000}, 20000);

  QueryOptions full;
  full.check_self_collision = true; // self-collision on the CPU (FCL) in both backends, honoring the ACM
  bench::run_table("DTC full query (robot-vs-env + robot-vs-self)", *fcl, *optix, robot, pool, full,
                   {10, 100, 1000}, 20000);

  // ---- cost decomposition: self vs environment vs the shared floor ----------------------------
  // The same robot with NO environment isolates the self pass. On the OptiX backend an empty
  // environment means no GAS and hence no launch at all (`do_gpu` is false), so its A/C rows are
  // the pure host path — read the OptiX block for the OVERLAP story (how much of B and C hides
  // inside D), and the FCL block for the per-phase cost anatomy.
  std::printf("\n########## COST DECOMPOSITION (self vs environment) ##########\n");
  print_anatomy(*model, robot, env, meshes);

  const SceneDescription empty_env;
  const auto fcl_bare = make_static_scene(model, empty_env, BackendHint::ForceCpuFcl, meshes);
  const auto optix_bare = make_static_scene(model, empty_env, BackendHint::ForceOptix, meshes);

  for (const int batch : {100, 1000, 10000}) {
    decompose("FCL", *fcl_bare, *fcl, robot, pool, batch, 20000);
    decompose("OptiX", *optix_bare, *optix, robot, pool, batch, 20000);
  }
  return 0;
}
