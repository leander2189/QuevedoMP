// benchmarks/bench_dtc — FCL vs OptiX on the real DTC cell: the rbrobout robot (UR10e on an Ewellix
// 900 mm lift, with the ee_hilok end-effector) against the work-object environment (mesh.stl + the
// fiducial markers) at their app world poses. Random 7-DOF poses, batch-size latency/throughput
// sweep — the same harness as bench_collision, on a real industrial scene instead of a synthetic
// wall.
//
// The work object is a LOCALIZED obstacle far from the base (only the extended arm + EE reach it), so
// this is the regime the opt-in robot-link broadphase cull targets. A/B it by running the binary
// twice: once plain, once with QUEVEDOMP_OPTIX_CULL=1. Build under the bench-optix preset.
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/robot/robot_instance.hpp"

#include "bench_harness.hpp"
#include "dtc_scene.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

int main() {
  if (!optix_available()) {
    std::printf("OptiX backend not built — configure with the bench-optix preset.\n");
    return 1;
  }
  const std::string fx = QUEVEDOMP_FIXTURE_DIR;

  const auto model = dtc::load_robot(fx);
  RobotInstance robot(model);
  dtc::load_acm(fx, robot.acm()); // SRDF allowed-collision matrix (for the full-query pass)
  const auto meshes = dtc::meshes(fx);
  const SceneDescription env = dtc::make_env(fx);

  const auto fcl = make_static_scene(model, env, BackendHint::ForceCpuFcl, meshes);
  const auto optix = make_static_scene(model, env, BackendHint::ForceOptix, meshes);

  Rng rng(2024);
  const std::vector<JointPosition> pool = dtc::sample_configs(*model, rng, 10000);

  const bool cull = std::getenv("QUEVEDOMP_OPTIX_CULL") != nullptr;
  std::printf("DTC cell: rbrobout (UR10e + Ewellix lift + ee_hilok, dof=%zu) vs work object "
              "(%zu env meshes)\n",
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
  return 0;
}
