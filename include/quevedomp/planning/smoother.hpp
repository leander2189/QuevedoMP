// planning/Smoother — post-planning path smoothing (Task 3.3, spec §6 Phase 3).
//
// A sampling planner returns a *feasible* path (often jagged); the smoother makes it usable. v0
// ships the iterative shortcut smoother (`ShortcutSmoother`); `BSplineSmoother` is deferred. Like
// the planner, all collision goes through the batch-first CollisionScene API and smoothing is
// deterministic per seed. Smoothing is the "polished" budget of the performance contract (item 2),
// separate from the planner's fast-to-feasible budget.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::planning {

struct SmootherParams {
  // Edge discretization for validating a shortcut (rad, max per-joint step) — match the planner's
  // so a shortcut that passes here is free at the same fidelity the planner guaranteed. Ignored
  // when max_link_sweep > 0.
  double edge_resolution = 0.05;

  // Cartesian-bounded shortcut validation (Task 3.3d P3) — match the planner's max_link_sweep /
  // lever_weights (see PlannerParams). 0 = off; empty weights are computed from the model by
  // make_shortcut_smoother (precompute via collision::cartesian_lever_weights for robots whose
  // mesh URIs need package dirs).
  double max_link_sweep = 0.0;
  JointPosition lever_weights;

  // Number of shortcut attempts. The real polish budget is time-based in the pipeline (Task 3.5);
  // this bounds work in isolation.
  std::size_t max_iterations = 200;

  // RNG seed for choosing shortcut endpoints — same seed ⇒ same smoothed path (determinism).
  std::uint64_t seed = 0;

  // Collision options for shortcut validation (boolean by default; must match how the path was
  // planned so padding/margins stay consistent).
  collision::QueryOptions collision;
};

// A path smoother. `smooth` is a pure function of (input, params, scene): const, reentrant, and
// deterministic. The output is collision-free (every retained segment was either in the free input
// or a validated shortcut) and never longer than the input (each shortcut replaces a sub-polyline
// with its chord — non-increasing by the triangle inequality).
class Smoother {
public:
  virtual ~Smoother() = default;
  [[nodiscard]] virtual Path smooth(const Path &path) const = 0;
};

// Iterative shortcut smoother over `robot` (its ACM) and `scene`. Throws std::runtime_error if
// either is null.
[[nodiscard]] std::unique_ptr<Smoother>
make_shortcut_smoother(SmootherParams params, std::shared_ptr<const RobotInstance> robot,
                       std::shared_ptr<const collision::CollisionScene> scene);

} // namespace quevedomp::planning
