// planning/PrmPlanner — a PRM-flavored multi-query roadmap planner (roadmap R5, build-plan Task
// 3.2 "quasi-static cells make a per-cell roadmap a natural candidate").
//
// The quasi-static-environment assumption (the same one the GAS design and the ClearanceField
// exploit) pays off differently here: a roadmap is built ONCE over the static scene — sampling free
// configs and validating candidate edges in UNBOUNDED fat batches, exactly the shape where the GPU
// backend finally wins outright — and then each query is cheap: connect start/goal to the roadmap,
// A* over the graph, P6 shortcut smoothing. Target: single-digit ms per query.
//
// It is a first-class Planner (plan(problem) → result), selected explicitly — never a fallback. The
// roadmap is built at make_prm_planner() time and the returned planner is const + reentrant, so one
// instance answers many concurrent queries (the multi-query point). Like the refiner (ADR-019), it
// gets a dedicated factory rather than the make_planner() string registry: PRM's construction
// config (roadmap size, connectivity) does not fit the flat PlannerParams, and building the roadmap
// is heavy work that belongs at construction, not hidden in a registry lookup. "prm" is still
// registered so it is discoverable and selecting it the wrong way fails loudly. See ADR-020.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/planning/planner.hpp"
#include "quevedomp/planning/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"

namespace quevedomp::planning {

struct PrmParams {
  // Roadmap sample budget: this many candidate configs are drawn and validated in fat batches;
  // the free ones become nodes (so in clutter the roadmap is sparser — num_nodes is the budget,
  // not a guaranteed free count). Larger ⇒ denser roadmap, better queries, heavier build.
  std::size_t num_nodes = 1000;

  // Each node connects to its k nearest neighbours (joint-space L2). connection_radius > 0 also
  // links every node within that radius (rad) — the union is validated. k caps the degree so a
  // dense cluster does not explode the edge batch.
  std::size_t k_neighbors = 10;
  double connection_radius = 0.0;

  // Candidate-edge validation batching: edges are grouped so each query_batch stays under about
  // this many configs (keeps a huge roadmap's edge validation off a single oversized allocation
  // while still fat enough for the GPU). 0 ⇒ one batch for everything.
  std::size_t edge_batch_configs = 200000;

  // Edge discretization for validating candidate edges (see PlannerParams). max_link_sweep > 0
  // (Task 3.3d P3) overrides edge_resolution; empty lever_weights are computed by make_prm_planner.
  double edge_resolution = 0.05;
  double max_link_sweep = 0.0;
  JointPosition lever_weights;

  // Sampling-space narrowing applied at BUILD time (intersected with the URDF limits), exactly
  // like PlanningProblem/Constraints. The roadmap is bound to these bounds.
  Constraints constraints;

  // Collision options the WHOLE roadmap is validated under (nodes, edges, and query-time
  // start/goal connections). The roadmap fixes the collision semantics: a query's own
  // PlanningProblem::collision is NOT used by PRM — it would invalidate the prevalidated edges.
  collision::QueryOptions collision;

  // Construction determinism: the roadmap is a pure function of (params, robot, scene, seed).
  std::uint64_t seed = 0;

  // Shortcut-smooth the extracted path (P6) before returning. Off ⇒ the raw graph path.
  bool smooth = true;
};

// What the one-time roadmap build produced (diagnostics; the studio/benchmark show these).
struct PrmBuildStats {
  std::size_t nodes = 0;               // free roadmap nodes kept
  std::size_t node_candidates = 0;     // configs sampled (num_nodes)
  std::size_t edges = 0;               // collision-free edges kept
  std::size_t edge_candidates = 0;     // candidate edges validated
  std::uint64_t collision_configs = 0; // total configs pushed through the backend at build
  double build_seconds = 0.0;
};

// Build the roadmap over `robot` (its model + ACM) and `scene`, returning a queryable Planner.
// Throws std::runtime_error if robot/scene is null or a bad sweep configuration is given. If
// `out_stats` is non-null it receives the build diagnostics. The build is the expensive step
// (fat-batch node + edge validation); plan() is then cheap and reentrant.
[[nodiscard]] std::unique_ptr<Planner>
make_prm_planner(PrmParams params, std::shared_ptr<const RobotInstance> robot,
                 std::shared_ptr<const collision::CollisionScene> scene,
                 PrmBuildStats *out_stats = nullptr);

} // namespace quevedomp::planning
