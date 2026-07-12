// collision/edge_discretization — Cartesian-bounded edge stepping (Task 3.3d P3).
//
// The uniform per-joint edge resolution (rad) states the discretization guarantee in the wrong
// space: 0.01 rad of a wrist joint moves the tool millimetres, 0.01 rad of the base pan sweeps
// the whole arm. Cartesian-bounded stepping states it in workspace terms instead: consecutive
// edge samples are chosen so that NO point of the robot's collision geometry moves more than
// `max_link_sweep` metres between them — the step count satisfies Σ_i w_i·|Δq_i| ≤ d per step,
// where w_i is a conservative per-joint lever bound. Low-lever joints get coarser (fewer wasted
// configs), high-lever joints finer, and the guarantee ("≤ 5 mm sweep") is scene-meaningful.
#pragma once

#include "quevedomp/collision/collision_scene.hpp" // MeshSources
#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp::collision {

// Per-dof lever weights w_i: an upper bound on how far ANY point of the robot's collision
// geometry can move per unit motion of joint i, over ALL configurations (m/rad for revolute and
// continuous joints, exactly 1 m/m for prismatic). Computed by a tip→base recursion over bounding
// balls centred on joint origins: a revolute rotation (URDF axes pass through the joint origin)
// keeps its distal ball radius invariant, a prismatic joint inflates it by its max travel, and
// link geometry contributes its exact extent (mesh URIs resolved + loaded via `meshes`, the same
// way make_static_scene does — throws on an unresolvable/unloadable URI, never silently skips).
// A movable joint with no distal collision geometry gets weight 0 (moving it sweeps nothing).
[[nodiscard]] JointPosition cartesian_lever_weights(const RobotModel &model,
                                                    const MeshSources &meshes = {});

// How an edge q0→q1 is split into steps(q1 - q0) uniform sub-segments for collision checking.
struct EdgeDiscretization {
  // Uniform fallback: max per-joint step (rad) — the pre-P3 behavior. Applies when
  // max_link_sweep == 0.
  double joint_resolution = 0.05;

  // > 0 (metres) enables Cartesian-bounded stepping: Σ w_i·|Δq_i| ≤ max_link_sweep per step.
  // Requires lever_weights of size dof.
  double max_link_sweep = 0.0;
  JointPosition lever_weights;

  // Number of sub-segments (≥ 1) for an edge with joint delta `delta`. Throws std::runtime_error
  // if max_link_sweep > 0 but lever_weights does not match delta's size.
  [[nodiscard]] int steps(const JointPosition &delta) const;
};

// Build the policy planners/smoothers run edges through, validating the params once up front:
// max_link_sweep == 0 ⇒ the uniform joint_resolution policy; > 0 with empty weights ⇒ weights are
// computed here from the model (with NO mesh sources — a robot whose mesh URIs need package dirs
// must precompute them via cartesian_lever_weights(model, meshes) and pass them in); > 0 with
// weights of the wrong size ⇒ throws.
[[nodiscard]] EdgeDiscretization make_edge_discretization(double joint_resolution,
                                                          double max_link_sweep,
                                                          const JointPosition &lever_weights,
                                                          const RobotModel &model);

} // namespace quevedomp::collision
