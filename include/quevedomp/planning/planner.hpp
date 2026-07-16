// planning/Planner — the algorithm-agnostic planner interface + execution-time selection
// (Task 3.2, spec §6 Phase 3). A planner turns a PlanningProblem into a PlanningResult, checking
// collisions ONLY through the batch-first CollisionScene API (the performance contract in the
// build plan). Concrete algorithms register under a string id; make_planner() picks one at
// execution time — there is no automatic algorithm selection and no silent fallback.
#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::planning {

// Planner configuration. A flat struct for v0 (RRT-Connect only); the spec's std::variant of
// algorithm-specific options (§3.4) is deferred until a second algorithm needs incompatible
// params (YAGNI). `algorithm` is the registry id make_planner() dispatches on.
struct PlannerParams {
  std::string algorithm = "rrt_connect";

  // Edge discretization handed to the collision backend: max per-joint step (rad) between the
  // samples of one edge. Smaller ⇒ finer (safer, more configs per edge). Ignored when
  // max_link_sweep > 0.
  double edge_resolution = 0.05;

  // Cartesian-bounded edge stepping (Task 3.3d P3): when > 0 (metres), the edge step count is
  // chosen so that no point of the robot's collision geometry moves more than this between
  // consecutive samples (Σ w_i·|Δq_i| ≤ max_link_sweep), replacing the uniform per-joint
  // edge_resolution — a workspace-stated guarantee ("≤ 5 mm sweep") with fewer wasted configs on
  // low-lever joints. 0 = off (edge_resolution applies).
  double max_link_sweep = 0.0;

  // Per-dof lever weights for max_link_sweep (see collision::cartesian_lever_weights). Leave
  // empty to have make_planner compute them from the model — robots whose mesh collision URIs
  // need package dirs must precompute with cartesian_lever_weights(model, meshes) and set this.
  JointPosition lever_weights;

  // RRT extension step (rad): how far a tree node reaches toward a sample in joint space.
  double max_extension = 0.5;

  // Fraction of samples drawn from the goal region instead of uniformly (goal biasing).
  double goal_bias = 0.05;

  // Candidate extensions gathered per growth step and validated as ONE query_batch — the lever
  // that keeps collision batches fat enough to engage the GPU (contract item 1). 1 ⇒ classic
  // serial RRT (thin batches, below the hybrid Auto crossover).
  std::size_t batch_size = 64;

  // Defensive cap on growth iterations; the real budget is PlanningProblem::timeout.
  std::size_t max_iterations = 100000;

  // Debug/visualization (roadmap R2): copy the final search trees into PlanningResult::trees
  // when planning ends. One copy at exit — zero cost in the growth loop; off by default.
  bool record_tree = false;
};

// The planning interface (spec §6). `plan` is const + reentrant: it owns no mutable state between
// calls and makes its own per-call Workspace, so distinct threads may plan concurrently on one
// planner (contract item 4). Determinism is per seed (contract item 3).
class Planner {
public:
  virtual ~Planner() = default;
  [[nodiscard]] virtual PlanningResult plan(const PlanningProblem &problem) const = 0;
};

// Build the planner named by `params.algorithm` over `robot` (its model + ACM) and `scene`.
// Throws std::runtime_error if the id names no registered planner (a clear error, never a silent
// fallback — contract). `robot`/`scene` must be non-null and share the same RobotModel.
//
// Note (deviation from spec §6's `shared_ptr<const RobotModel>`): the planner takes a
// RobotInstance because collision queries need the ACM it carries (self-collision honors it); the
// model is reached via robot->model_ptr(). Mechanically required, so the plan wins here.
[[nodiscard]] std::unique_ptr<Planner>
make_planner(const PlannerParams &params, std::shared_ptr<const RobotInstance> robot,
             std::shared_ptr<const collision::CollisionScene> scene);

// The ids make_planner() accepts, for diagnostics and tests.
[[nodiscard]] std::vector<std::string> registered_planners();

} // namespace quevedomp::planning
