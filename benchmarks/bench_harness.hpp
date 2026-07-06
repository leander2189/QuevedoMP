// benchmarks/bench_harness — shared FCL-vs-OptiX timing table, used by bench_collision and bench_dtc.
//
// Methodology: scenes/workspaces are built ONCE by the caller (context + pipeline + GAS + FK-ray
// precompute are one-time setup, not per-query). Per batch size we warm up both backends (lazy CUDA
// init, first buffer growth) and then time the steady-state hot path, taking the MINIMUM per-call
// time over several trials — the minimum rejects the one-off jitter (scheduling, clock ramp,
// contention) that inflates a mean. Build under a Release, sanitizers-OFF preset (bench-optix): ASan
// would slow the CPU/FCL path enough to make the comparison meaningless.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::bench {

inline double fraction_true(const std::vector<std::uint8_t> &v) {
  if (v.empty())
    return 0.0;
  std::size_t c = 0;
  for (std::uint8_t x : v)
    c += x ? 1 : 0;
  return static_cast<double>(c) / static_cast<double>(v.size());
}

// Best (minimum) per-call time in ms over 5 trials of `reps` iterations each.
template <class F> double time_ms(int reps, F &&f) {
  double best = 1e30;
  for (int t = 0; t < 5; ++t) {
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < reps; ++i)
      f();
    const auto t1 = std::chrono::steady_clock::now();
    best = std::min(best, std::chrono::duration<double, std::milli>(t1 - t0).count() / reps);
  }
  return best;
}

// Time FCL vs OptiX query_batch over a set of batch sizes drawn from `pool`, printing a table of
// per-batch latency, throughput, and speedup. rep_budget/batch sets the per-size repetition count.
inline void run_table(const char *title, const collision::CollisionScene &fcl,
                      const collision::CollisionScene &optix, const RobotInstance &robot,
                      const std::vector<JointPosition> &pool, const collision::QueryOptions &opts,
                      const std::vector<int> &batches, int rep_budget) {
  auto fcl_ws = fcl.make_workspace();
  auto optix_ws = optix.make_workspace();

  std::printf("\n== %s ==\n", title);
  std::printf("%8s | %10s | %10s | %12s | %12s | %8s\n", "batch", "FCL ms", "OptiX ms", "FCL cfg/s",
              "OptiX cfg/s", "speedup");
  std::printf("---------+------------+------------+--------------+--------------+---------\n");

  for (int batch : batches) {
    if (batch > static_cast<int>(pool.size()))
      break;
    const std::span<const JointPosition> qs(pool.data(), static_cast<std::size_t>(batch));

    const collision::BatchResult wf = fcl.query_batch(robot, qs, opts, *fcl_ws);
    const collision::BatchResult wo = optix.query_batch(robot, qs, opts, *optix_ws);
    (void)wf;
    (void)wo;

    const int reps = std::max(3, rep_budget / batch);
    const double fcl_ms = time_ms(reps, [&] { fcl.query_batch(robot, qs, opts, *fcl_ws); });
    const double optix_ms = time_ms(reps, [&] { optix.query_batch(robot, qs, opts, *optix_ws); });

    const double fcl_cps = batch / (fcl_ms / 1e3);
    const double optix_cps = batch / (optix_ms / 1e3);
    std::printf("%8d | %10.3f | %10.3f | %12.0f | %12.0f | %7.2fx\n", batch, fcl_ms, optix_ms,
                fcl_cps, optix_cps, fcl_ms / optix_ms);
  }
}

} // namespace quevedomp::bench
