// planning/TrajectoryRefiner — optimization-based trajectory refiner (roadmap R4, build-plan
// Task 3.3c). A CHOMP/TrajOpt-flavored gradient optimizer that pulls a trajectory toward smooth,
// high-clearance motion using the R3 ClearanceField (ADR-018) for obstacle gradients, then
// CERTIFIES the result collision-free through the exact CollisionScene backend — the field is
// approximate, so it is never the certificate (ADR-018's standing rule).
//
// It is a first-class Planner (implements plan(problem) → result), selected explicitly like any
// other — NEVER an automatic fallback (performance contract). Two composition modes:
//   • Refiner   — seeded with a feasible trajectory (`RefinerParams::seed`), the pipeline's polish
//                 stage; this is where "smooth and logical" is manufactured.
//   • Standalone— seed empty ⇒ a straight-line guess from the problem's start to a resolved goal
//                 config; fast when it works, honest failure (NoSolution) on a local minimum.
// The mode that ran is reported in PlanningResult::message + PlanningStats::refiner_mode.
//
// Why a dedicated make_refiner() factory rather than the make_planner() string registry (the RRT
// route): the refiner needs two dependencies the registry's fixed (params, robot, scene) signature
// cannot carry — a built ClearanceField and the robot sphere cover. This mirrors make_shortcut_
// smoother(), which is likewise its own factory. `make_planner("chomp", …)` is still registered,
// but it throws a directive error pointing here (discoverable via registered_planners(), never a
// silent fallback). See ADR-019.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "quevedomp/clearance/clearance_field.hpp"
#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::planning {

struct RefinerParams {
  // Seed trajectory to polish (refiner mode). Empty ⇒ standalone mode: a straight line from
  // problem.start to a resolved goal config. When non-empty, its endpoints are the fixed
  // start/goal of the optimization (they must match problem.start / satisfy the goal — the
  // refiner does not re-plan around them).
  Path seed;

  // Waypoint count of the optimized trajectory: the seed / straight line is resampled to this
  // many configs (endpoints fixed, the interior optimized). More ⇒ finer clearance/smoothness
  // resolution and a fatter per-iteration clearance batch. Minimum 3 (two endpoints + interior).
  std::size_t waypoints = 64;

  // CHOMP iteration budget. The optimizer also stops early on convergence (see convergence_tol)
  // or when problem.timeout elapses.
  std::size_t max_iterations = 100;

  // Cost weights. Smoothness pulls toward minimum summed-squared acceleration; clearance pushes
  // configs out of the ε-shell around obstacles.
  double smoothness_weight = 1.0;
  double clearance_weight = 1.0;

  // CHOMP obstacle hinge width (m): the cost is 0 beyond ε clearance, quadratic in [0, ε], and
  // linear when penetrating (d < 0). Sized around the field's voxel resolution + robot margin.
  double clearance_epsilon = 0.10;

  // Update step (learning rate) 1/η applied to the A⁻¹-preconditioned gradient. Smaller ⇒ safer,
  // slower. The A⁻¹ (smoothness-metric) preconditioning is what keeps obstacle pushes from
  // kinking the path — plain descent on the obstacle term alone would jag.
  double step_size = 0.1;

  // Convergence: stop once the largest per-waypoint update (joint-space L2) falls below this (rad).
  double convergence_tol = 1e-4;

  // ---- Certificate re-validation (exact backend) -------------------------------------------
  // Edge discretization for the final CollisionScene certificate — match the planner that
  // produced the seed so a certified edge is free at the same fidelity. Ignored when
  // max_link_sweep > 0 (Task 3.3d P3); empty lever_weights are computed from the model by
  // make_refiner (precompute for robots whose mesh URIs need package dirs).
  double edge_resolution = 0.05;
  double max_link_sweep = 0.0;
  JointPosition lever_weights;

  // Collision options for the certificate (must match how the seed was planned so padding /
  // margins / self-collision stay consistent).
  collision::QueryOptions collision;

  // Seed for the standalone goal IK (determinism per seed). CHOMP itself is RNG-free. Named
  // distinctly from the `seed` trajectory above.
  std::uint64_t rng_seed = 0;
};

// Build a refiner over `robot` (its model + ACM), `scene` (the exact certificate backend), and
// `field` (the R3 clearance SDF of the SAME static environment `scene` holds) with the robot
// `spheres` cover (see clearance::decompose_robot). Throws std::runtime_error if robot/scene/field
// is null, spheres is empty, or a bad sweep configuration is given (validated once here, never
// mid-plan). Returns a Planner; call plan(problem) to refine.
[[nodiscard]] std::unique_ptr<Planner>
make_refiner(RefinerParams params, std::shared_ptr<const RobotInstance> robot,
             std::shared_ptr<const collision::CollisionScene> scene,
             std::shared_ptr<const clearance::ClearanceField> field,
             clearance::RobotSpheres spheres);

} // namespace quevedomp::planning
